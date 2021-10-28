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
    f32::consts::{FRAC_PI_8, PI},
    time::{Duration, Instant},
};

mod chassis;
mod lidar;
mod rtk;

use chassis::Chassis;
use lidar::Lidar;

pub use lidar_faselase::PointZipped;
pub use pm1_sdk::{
    model::{Odometry, Physical},
    PM1Status,
};
pub use rtk_ins570::{Enu, WGS84};

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

#[derive(Clone)]
pub struct Robot {
    chassis: Chassis,
    lidar: Lidar,
    event: Sender<Event>,

    artifical_deadline: Arc<Mutex<Instant>>,
    tracker: Arc<Mutex<Tracker>>,
}

pub enum Event {
    ConnectionModified(u32),
    ChassisStatusUpdated(PM1Status),
    ChassisOdometerUpdated(f32, f32),
    PoseUpdated(Pose),
    LidarFrameEncoded(Vec<u8>),
    CollisionDetected(f32),
}

type Trajectory = Box<dyn Iterator<Item = (Duration, Odometry)> + Send>;

struct CollisionInfo(pub Duration, pub Odometry, pub f32);

const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(500);

impl From<Isometry2<f32>> for Pose {
    fn from(src: Isometry2<f32>) -> Self {
        Self {
            x: src.translation.vector[0],
            y: src.translation.vector[1],
            theta: src.rotation.angle(),
        }
    }
}

impl Pose {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };

    pub(crate) fn transform_u16(&self, len: u16, dir: u16) -> (f32, f32) {
        let Pose { x, y, theta } = self;
        let len = len as f32 / 100.0;
        let dir = dir as f32 * 2.0 * PI / 5760.0 + theta;
        let (sin, cos) = dir.sin_cos();
        (cos * len + x, sin * len + y)
    }

    #[inline]
    pub fn transform_zipped(&self, p: PointZipped) -> (f32, f32) {
        self.transform_u16(p.len(), p.dir())
    }
}

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
        let connections = Arc::new(Mutex::new(0u32));

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
            let connections = connections.clone();
            task::spawn(async move {
                while let Ok(e) = rtk.recv().await {
                    use rtk::Event::*;
                    match e {
                        Connected => {
                            let mut lock = connections.lock().await;
                            if *lock & 0b10 == 0 {
                                *lock |= 0b10;
                                send_async!(Event::ConnectionModified(*lock) => robot.event).await;
                            }
                        }
                        Disconnected => {
                            let mut lock = connections.lock().await;
                            if *lock & 0b10 == 1 {
                                *lock &= !0b10;
                                send_async!(Event::ConnectionModified(*lock) => robot.event).await;
                            }
                        }
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
                                    let mut lock = connections.lock().await;
                                    if *lock & 0b100 == 0 {
                                        *lock |= !0b100;
                                        send_async!(Event::ConnectionModified(*lock) => robot.event).await;
                                    }
                                    join!(
                                        send_async!(Event::PoseUpdated(pose.into()) => robot.event),
                                        robot.automitic(pose),
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
                        Connected => {
                            let mut lock = connections.lock().await;
                            if *lock & 0b1 == 1 {
                                *lock &= !0b1;
                                send_async!(Event::ConnectionModified(*lock) => robot.event).await;
                            }
                        }
                        Disconnected => {
                            let mut lock = connections.lock().await;
                            if *lock & 0b1 == 1 {
                                *lock &= !0b1;
                                send_async!(Event::ConnectionModified(*lock) => robot.event).await;
                            }
                        }
                        StatusUpdated(s) => {
                            send_async!(Event::ChassisStatusUpdated(s) => robot.event).await;
                        }
                        OdometryUpdated(t, o) => {
                            let pose = filter.lock().await.update(PoseType::Relative, t, o.pose);
                            join!(
                                send_async!(Event::ChassisOdometerUpdated(o.s, o.a) => robot.event),
                                send_async!(Event::PoseUpdated(pose.into()) => robot.event),
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
