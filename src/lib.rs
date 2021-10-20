use async_std::{
    channel::{unbounded, Receiver, Sender},
    sync::{Arc, Mutex},
    task,
};
use futures::join;
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

use chassis::Chassis;
use lidar::Lidar;

pub use pm1_sdk::{
    model::{Odometry, Physical},
    PM1Status,
};

type Trajectory = Box<dyn Iterator<Item = (Duration, Odometry)> + Send>;

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
        let (chassis, from_chassis) = Chassis::supervisor();
        let (lidar, from_lidar) = Lidar::supervisor();
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

    pub async fn drive(&self, target: Physical) {
        join!(
            async {
                let deadline = Instant::now() + ARTIFICIAL_TIMEOUT;
                *self.artifical_deadline.lock().await = deadline;
            },
            self.check_and_drive(target)
        );
    }

    pub async fn read(&self) -> Option<Vec<Isometry2<f32>>> {
        self.tracker.lock().await.read("default").ok()
    }

    pub async fn record(&self) {
        let _ = self.tracker.lock().await.record_to("default");
    }

    pub async fn track(&self) {
        let _ = self.tracker.lock().await.track("default");
    }

    pub async fn stop(&self) {
        self.tracker.lock().await.stop_task();
    }

    pub async fn set_pause(&self, value: bool) {
        self.tracker.lock().await.pause = value;
    }

    async fn automitic(&self, pose: Isometry2<f32>) {
        if let Some(frac) = self.tracker.lock().await.put_pose(&pose) {
            if *self.artifical_deadline.lock().await < Instant::now() {
                self.check_and_drive(Physical {
                    speed: 0.25,
                    rudder: -2.0 * (0.5 - frac),
                })
                .await;
            }
        }
    }

    async fn check_and_drive(&self, mut p: Physical) {
        if p.speed == 0.0 {
            self.drive_and_warn(p, 0.0).await;
        } else {
            // 轨迹预测
            if let Some(tr) = self.chassis.predict(p).await {
                // 碰撞预警
                match self.lidar.check(tr).await {
                    // 可能碰撞
                    Some(CollisionInfo(time, Odometry { s, a, pose: _ }, level)) => {
                        if s < 0.20 && a < FRAC_PI_8 {
                            // 将在极小距离内碰撞
                            self.drive_and_warn(Physical::RELEASED, 1.0).await;
                        } else {
                            // 一般碰撞
                            let sec = time.as_secs_f32();
                            p.speed *= sec / 2.0;
                            self.drive_and_warn(p, f32::min(1.0, (2.0 - sec) * level))
                                .await;
                        }
                    }
                    // 不可能碰撞
                    None => self.drive_and_warn(p, 0.0).await,
                };
            }
        }
    }

    #[inline]
    async fn drive_and_warn(&self, p: Physical, r: f32) {
        join!(
            self.chassis.drive(p),
            send_async!(Event::CollisionDetected(r) => self.event),
        );
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

    macro_rules! join_async {
        ($($tokens:tt)*) => {
            async_std::task::block_on(async {
                futures::join! {
                    $( $tokens )*
                }
            });
        };
    }

    pub(crate) use {join_async, send_async};
}
