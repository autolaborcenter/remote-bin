use crate::{DeviceCode, Odometry, Physical, Pose};
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
    f32::consts::{FRAC_PI_2, FRAC_PI_8},
    sync::atomic::{AtomicU32, Ordering::Relaxed},
    time::{Duration, Instant},
};

mod chassis;
mod lidar;
mod rtk;

#[cfg(feature = "wired-joystick")]
mod joystick;

use chassis::Chassis;
use lidar::Lidar;

pub use pm1_sdk::PM1Status;
pub type Trajectory = Box<dyn Iterator<Item = (Duration, Odometry)> + Send>;

#[derive(Clone)]
pub struct Robot {
    chassis: Chassis,
    lidar: Lidar,
    event: Sender<Event>,

    artificial_deadline: Arc<Mutex<Instant>>,
    joystick_deadline: Arc<Mutex<Instant>>,
    tracking_speed: Arc<AtomicU32>,
    tracker: Arc<Mutex<Tracker>>,
}

pub enum Event {
    ConnectionModified(DeviceCode),
    ChassisStatusUpdated(PM1Status),
    ChassisOdometerUpdated(f32, f32),
    PoseUpdated(Pose),
    LidarFrameEncoded(Vec<u8>),
    CollisionDetected(f32),
}

struct CollisionInfo {
    pub time: Duration,
    pub pose: Odometry,
    pub risk: f32,
    pub force: Vector2<f32>,
}

const JOYSTICK_TIMEOUT: Duration = Duration::from_millis(500); //// 手柄控制保护期
const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(500); // 人工控制保护期
const ACTIVE_COLLISION_AVOIDING: f32 = 2.5; // ----------------- // 主动避障强度

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
        let device_code = Arc::new(Mutex::new(DeviceCode::default()));

        let now = Instant::now();
        let robot = Self {
            chassis,
            lidar,
            event,
            joystick_deadline: Arc::new(Mutex::new(now)),
            artificial_deadline: Arc::new(Mutex::new(now)),
            tracking_speed: Arc::new(AtomicU32::new(0f32.to_bits())),
            tracker: Arc::new(Mutex::new(Tracker::new("path").unwrap())),
        };

        let filter = Arc::new(Mutex::new(InterpolationAndPredictionFilter::new()));
        {
            let filter = filter.clone();
            let robot = robot.clone();
            let device_code = device_code.clone();
            task::spawn(async move {
                let mut state_mem = Default::default();
                while let Ok(e) = rtk.recv().await {
                    use rtk::Event::*;
                    match e {
                        Connected => {
                            if let Some(code) = device_code.lock().await.set(&[2]) {
                                send_async!(Event::ConnectionModified(code) => robot.event).await;
                            }
                        }
                        Disconnected => {
                            if let Some(code) = device_code.lock().await.clear(&[2, 4]) {
                                send_async!(Event::ConnectionModified(code) => robot.event).await;
                            }
                        }
                        SolutionUpdated(t, s) => match s {
                            Solution::Uninitialized(state) => {
                                if state != state_mem {
                                    state_mem = state;
                                    println!("{}", state);
                                }
                            }
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
                                    if let Some(code) = device_code.lock().await.set(&[3, 4]) {
                                        send_async!(Event::ConnectionModified(code) => robot.event)
                                            .await;
                                    }
                                    join!(
                                        send_async!(Event::PoseUpdated(pose.into()) => robot.event),
                                        robot.automitic(pose),
                                    );
                                } else if state != state_mem {
                                    if let Some(code) = device_code.lock().await.clear(&[4]) {
                                        send_async!(Event::ConnectionModified(code) => robot.event)
                                            .await;
                                    }
                                    state_mem = state;
                                    println!("{}", state);
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
                            if let Some(code) = device_code.lock().await.set(&[0]) {
                                send_async!(Event::ConnectionModified(code) => robot.event).await;
                            }
                        }
                        Disconnected => {
                            if let Some(code) = device_code.lock().await.clear(&[0, 1]) {
                                send_async!(Event::ConnectionModified(code) => robot.event).await;
                            }
                        }
                        StatusUpdated(s) => {
                            send_async!(Event::ChassisStatusUpdated(s) => robot.event).await;
                            if s.power_switch {
                                if let Some(code) = device_code.lock().await.set(&[1]) {
                                    send_async!(Event::ConnectionModified(code) => robot.event)
                                        .await;
                                }
                            } else {
                                if let Some(code) = device_code.lock().await.clear(&[1]) {
                                    send_async!(Event::ConnectionModified(code) => robot.event)
                                        .await;
                                }
                            }
                        }
                        OdometryUpdated(t, o) => {
                            let pose = filter.lock().await.update(PoseType::Relative, t, o.pose);
                            join!(
                                send_async!(Event::ChassisOdometerUpdated(o.s, o.a) => robot.event),
                                send_async!(Event::PoseUpdated(pose.into()) => robot.event),
                                robot.automitic(pose),
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
        #[cfg(feature = "wired-joystick")]
        {
            let robot = robot.clone();
            task::spawn_blocking(move || {
                let mut joystick = joystick::Joystick::new();
                loop {
                    let target = joystick.get();
                    if !target.is_released() {
                        task::block_on(async {
                            let deadline = Instant::now() + JOYSTICK_TIMEOUT;
                            *robot.joystick_deadline.lock().await = deadline;
                        });
                        task::block_on(robot.check_and_drive(target));
                        std::thread::sleep(Duration::from_millis(50));
                    } else {
                        std::thread::sleep(Duration::from_millis(400));
                    }
                }
            });
        }

        (robot.clone(), to_extern)
    }

    pub fn set_tracking_speed(&self, val: f32) {
        self.tracking_speed.store(val.to_bits(), Relaxed);
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

    pub async fn predict(&self) -> Trajectory {
        self.chassis.predict().await
    }

    pub async fn drive(&self, target: Physical) {
        #[cfg(not(feature = "wired-joystick"))]
        if *self.joystick_deadline.lock().await >= Instant::now() {
            return;
        }

        join!(
            async {
                let deadline = Instant::now() + ARTIFICIAL_TIMEOUT;
                *self.artificial_deadline.lock().await = deadline;
            },
            self.check_and_drive(target)
        );
    }

    async fn automitic(&self, pose: Isometry2<f32>) {
        #[cfg(not(feature = "wired-joystick"))]
        if *self.joystick_deadline.lock().await >= Instant::now() {
            return;
        }

        if let Some((speed, rudder)) = self.tracker.lock().await.put_pose(&pose) {
            if *self.artificial_deadline.lock().await < Instant::now() {
                self.check_and_drive(Physical {
                    speed: f32::from_bits(self.tracking_speed.load(Relaxed)) * speed,
                    rudder,
                })
                .await;
            }
        }
    }

    async fn check_and_drive(&self, mut p: Physical) {
        // 保存目标状态
        self.chassis.store_raw_target(p).await;
        // 目标是静止不动
        if p.is_static() {
            self.drive_and_warn(p, 0.0).await;
        }
        // 可能碰撞
        else if let Some(collision) = self.lidar.check(self.chassis.predict().await).await {
            let (p, r) = if collision.pose.s < 0.2 && collision.pose.a < FRAC_PI_8 {
                // 将在极小距离内碰撞
                (Physical::RELEASED, 1.0)
            } else {
                // 一般碰撞

                // 减速
                let sec = collision.time.as_secs_f32();
                p.speed *= sec / 2.0;
                // 转向
                let modifier = -collision.force[1].atan2(ACTIVE_COLLISION_AVOIDING);
                p.rudder = if modifier > 0.0 {
                    f32::min(p.rudder + modifier, FRAC_PI_2)
                } else {
                    f32::max(p.rudder + modifier, -FRAC_PI_2)
                };

                (p, f32::min(1.0, (2.0 - sec) * collision.risk))
            };
            self.drive_and_warn(p, r).await;
        }
        // 不可能碰撞
        else {
            self.drive_and_warn(p, 0.0).await;
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
