use async_std::{
    channel::{unbounded, Receiver, Sender},
    sync::{Arc, Mutex},
    task,
};
use chassis::Chassis;
use futures::join;
use lidar::Lidar;
use parry2d::na::{Isometry2, Vector2};
use path_tracking::Tracker;
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use rtk_ins570::{Solution, SolutionState};
use std::{
    f32::consts::FRAC_PI_8,
    time::{Duration, Instant},
};

mod chassis;
mod lidar;
mod rtk;

type Trajectory = Box<dyn Iterator<Item = (Duration, Odometry)> + Send>;

pub use pm1_sdk::{
    model::{Odometry, Physical},
    PM1Status,
};

struct CollisionInfo(pub Duration, pub Odometry, pub f32);

#[derive(Clone)]
pub struct Robot {
    chassis: Chassis,
    lidar: Lidar,
    event: Sender<Event>,

    artifical_deadline: Arc<Mutex<Instant>>,
    tracker: Arc<Mutex<Tracker>>,
}

pub enum Event {
    ChassisStatusUpdated(PM1Status),
    ChassisOdometerUpdated(f32, f32),
    PoseUpdated(Isometry2<f32>),
    LidarFrameEncoded(Vec<u8>),
    CollisionDetected(f32),
}

const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(500);

impl Robot {
    pub async fn spawn(rtk: bool) -> (Self, Receiver<Event>) {
        let rtk = if rtk {
            rtk::supervisor()
        } else {
            unbounded().1
        };
        let (chassis, from_chassis) = chassis::supervisor();
        let (lidar, from_lidar) = lidar::supervisor();
        let (event, to_extern) = unbounded();

        let robot = Self {
            chassis,
            lidar,
            event,
            artifical_deadline: Arc::new(Mutex::new(Instant::now())),
            tracker: Arc::new(Mutex::new(Tracker::new("path").unwrap())),
        };

        let filter = Arc::new(Mutex::new(InterpolationAndPredictionFilter::new()));
        {
            let filter = filter.clone();
            let robot = robot.clone();
            task::spawn(async move {
                while let Ok(e) = rtk.recv().await {
                    use rtk::Event::*;
                    match e {
                        Connected => {}
                        Disconnected => {}
                        SolutionUpdated(t, s) => match s {
                            Solution::Uninitialized(_) => {}
                            Solution::Data { state, enu, dir } => {
                                let SolutionState {
                                    state_pos,
                                    state_dir,
                                    satellites: _,
                                } = state;
                                if state_pos > 40 && state_dir > 30 {
                                    let pose = filter.lock().await.update(
                                        PoseType::Absolute,
                                        t,
                                        Isometry2::new(
                                            Vector2::new(enu.e as f32, enu.n as f32),
                                            dir as f32,
                                        ),
                                    );
                                    join!(
                                        send_async!(Event::PoseUpdated(pose) => robot.event),
                                        async { robot.automitic(pose).await }
                                    );
                                }
                            }
                        },
                    }
                }
            });
        }
        {
            let robot = robot.clone();
            // move: let filter = filter.clone();
            task::spawn(async move {
                while let Ok(e) = from_chassis.recv().await {
                    use chassis::Event::*;
                    match e {
                        Connected => {}
                        Disconnected => {}
                        StatusUpdated(s) => {
                            send_async!(Event::ChassisStatusUpdated(s) => robot.event).await;
                        }
                        OdometryUpdated(t, o) => {
                            let pose = filter.lock().await.update(PoseType::Relative, t, o.pose);
                            join!(
                                send_async!(Event::ChassisOdometerUpdated(o.s, o.a) => robot.event),
                                send_async!(Event::PoseUpdated(pose) => robot.event),
                                async { robot.automitic(pose).await }
                            );
                        }
                    }
                }
            });
        }
        {
            let event = robot.event.clone();
            task::spawn(async move {
                while let Ok(e) = from_lidar.recv().await {
                    use lidar::Event::*;
                    match e {
                        Connected => {}
                        Disconnected => {}
                        FrameEncoded(buf) => {
                            send_async!(Event::LidarFrameEncoded(buf) => event).await;
                        }
                    }
                }
            });
        }

        (robot.clone(), to_extern)
    }

    pub async fn drive(&self, mut target: Physical) {
        join!(
            async {
                let deadline = Instant::now() + ARTIFICIAL_TIMEOUT;
                *self.artifical_deadline.lock().await = deadline;
            },
            async {
                if target.speed == 0.0 {
                    self.drive_and_warn(target, 0.0).await;
                } else {
                    if let Some(tr) = self.chassis.predict(target).await {
                        if let Some(ci) = self.lidar.check(tr).await {
                            match ci {
                                Some(CollisionInfo(time, Odometry { s, a, pose: _ }, p)) => {
                                    if s < 0.20 && a < FRAC_PI_8 {
                                        self.drive_and_warn(Physical::RELEASED, 1.0).await;
                                    } else {
                                        let sec = time.as_secs_f32();
                                        target.speed *= sec / 2.0;
                                        self.drive_and_warn(target, f32::min(1.0, (2.0 - sec) * p))
                                            .await;
                                    }
                                }
                                None => self.drive_and_warn(target, 0.0).await,
                            };
                        }
                    }
                }
            }
        );
    }

    #[inline]
    async fn drive_and_warn(&self, p: Physical, r: f32) {
        join!(
            self.chassis.drive(p),
            send_async!(Event::CollisionDetected(r) => self.event),
        );
    }

    #[inline]
    async fn automitic(&self, pose: Isometry2<f32>) {
        if let Some(frac) = self.tracker.lock().await.put_pose(&pose) {
            if *self.artifical_deadline.lock().await < Instant::now() {
                self.drive(Physical {
                    speed: 0.25,
                    rudder: -2.0 * (0.5 - frac),
                })
                .await;
            }
        }
    }
}

use macros::*;
mod macros {
    macro_rules! send_async {
        ($msg:expr => $sender:expr) => {
            async {
                let _ = $sender.send($msg).await;
            }
        };
    }

    macro_rules! call {
        ($sender:expr; $command:expr, $value:expr) => {{
            let mut result = None;
            let (sender, receiver) = async_std::channel::bounded(1);
            if let Ok(_) = (&$sender).send($command($value, sender)).await {
                while let Ok(r) = receiver.recv().await {
                    result = Some(r);
                }
            }
            result
        }};
    }

    pub(crate) use {call, send_async};
}
