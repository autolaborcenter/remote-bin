﻿use crate::{DeviceCode, Odometry, Physical, Pose};
use async_std::{
    channel::{unbounded, Receiver, Sender},
    path::PathBuf,
    sync::{Arc, Mutex},
    task,
};
use futures::join;
use parry2d::na::{Isometry2, Point, Point2, Vector2};
use path_tracking::{Parameters, Path, PathFile, RecordFile, Sector, TrackContext, Tracker};
use pm1_sdk::model::{Pm1Model, Pm1Predictor, TrajectoryPredictor};
use pose_filter::{gaussian, ParticleFilter, ParticleFilterParameters};
use rtk_ins570::Solution;
use std::{
    f32::consts::{FRAC_PI_2, FRAC_PI_8, PI},
    sync::atomic::{AtomicU32, Ordering::Relaxed},
    time::{Duration, Instant},
};

mod chassis;
#[macro_use]
mod filter;
mod lidar;
mod rtk;

#[cfg(feature = "wired-joystick")]
mod joystick;

use chassis::Chassis;
use lidar::Lidar;

pub use pm1_sdk::PM1Status;
pub use rtk::reauth;
pub type Trajectory = Box<TrajectoryPredictor<Pm1Predictor>>;

#[derive(Clone)]
pub struct Robot {
    context_dir: PathBuf,
    chassis: Chassis,
    lidar: Lidar,
    event: Sender<Event>,

    artificial_deadline: Arc<Mutex<Instant>>,
    joystick_deadline: Arc<Mutex<Instant>>,
    tracking_speed: Arc<AtomicU32>,
    task: Arc<Mutex<Task>>,
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

enum Task {
    Idle,
    WaitingPose,
    Record(RecordFile),
    Track(Path, TrackContext),
}

#[cfg(feature = "wired-joystick")]
const JOYSTICK_TIMEOUT: Duration = Duration::from_millis(500); // 手柄控制保护期

const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(500); // 人工控制保护期
const ACTIVE_COLLISION_AVOIDING: f32 = 2.5; // ----------------- // 主动避障强度

impl Robot {
    pub async fn spawn(mut context_dir: PathBuf, rtk: bool) -> (Self, Receiver<Event>) {
        let rtk = if rtk {
            rtk::supervisor(context_dir.clone())
        } else {
            unbounded().1
        };
        let (chassis, from_chassis) = Chassis::supervisor();
        let (lidar, from_lidar) = Lidar::supervisor();
        let (event, to_extern) = unbounded();
        let device_code = Arc::new(Mutex::new(DeviceCode::default()));

        let now = Instant::now();
        context_dir.push("path");
        let robot = Self {
            context_dir,
            chassis,
            lidar,
            event,
            joystick_deadline: Arc::new(Mutex::new(now)),
            artificial_deadline: Arc::new(Mutex::new(now)),
            tracking_speed: Arc::new(AtomicU32::new(0f32.to_bits())),
            task: Arc::new(Mutex::new(Task::Idle)),
        };

        let filter = particle_filter!();
        let time_origin = Instant::now();
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
                            Solution::Data { state, enu, dir: _ } => {
                                if state.state_pos > 40 {
                                    filter.lock().await.measure(
                                        t - time_origin,
                                        point(enu.e as f32, enu.n as f32),
                                    );
                                    if let Some(code) = device_code.lock().await.set(&[3, 4]) {
                                        send_async!(Event::ConnectionModified(code) => robot.event)
                                            .await;
                                    }
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
                let mut s = 0.0;
                let mut a = 0.0;
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
                        WheelsUpdated(t, wheels) => {
                            let (pose, model) = {
                                let mut filter = filter.lock().await;
                                filter.update(t - time_origin, wheels);
                                update_wheel!(filter);
                                (filter.get(), filter.parameters.default_model.clone())
                            };
                            let odom = model.wheels_to_velocity(wheels).to_odometry();
                            s += odom.s;
                            a += odom.a;
                            join!(
                                send_async!(Event::ChassisOdometerUpdated(s, a) => robot.event),
                                send_async!(Event::PoseUpdated(pose.into()) => robot.event),
                                robot.chassis.update_model(model),
                                robot.automatic(pose),
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
                        task::block_on(robot.drive_and_warn(target, 0.0));
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

    pub async fn read_path(&self) -> Option<Vec<Isometry2<f32>>> {
        PathFile::open(self.context_dir.as_path())
            .await
            .ok()
            .map(|f| f.collect())
    }

    pub async fn record(&self) {
        *self.task.lock().await = Task::WaitingPose;
    }

    pub async fn track(&self) -> async_std::io::Result<()> {
        let to_search = Sector {
            radius: 4.0,
            angle: PI,
        };
        let path = path_tracking::Path::new(
            PathFile::open(self.context_dir.as_path()).await?,
            to_search,
            10,
        );
        *self.task.lock().await = Task::Track(
            path,
            TrackContext::new(Parameters {
                search_range: to_search,
                light_radius: 0.6,
                r#loop: false,
            }),
        );
        Ok(())
    }

    pub async fn stop(&self) {
        *self.task.lock().await = Task::Idle;
    }

    pub async fn predict(&self) -> Option<Trajectory> {
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

    async fn automatic(&self, pose: Isometry2<f32>) {
        let mut task = self.task.lock().await;
        match &mut *task {
            Task::Idle => {}
            Task::WaitingPose => {
                if let Ok(r) = RecordFile::new(self.context_dir.as_path(), pose).await {
                    *task = Task::Record(r);
                }
            }
            Task::Record(recorder) => {
                if let Ok(true) = recorder.record(pose).await {
                    println!(
                        "saved: {:3} {:3} | {:1}°",
                        pose.translation.vector[0],
                        pose.translation.vector[1],
                        pose.rotation.angle().to_degrees()
                    )
                }
            }
            Task::Track(path, context) => {
                #[cfg(not(feature = "wired-joystick"))]
                if *self.joystick_deadline.lock().await >= Instant::now() {
                    return;
                }
                if *self.artificial_deadline.lock().await >= Instant::now() {
                    return;
                }
                let clone = context.clone();
                let mut tracker = Tracker {
                    path: &path,
                    context: clone,
                };
                if let Ok((k, rudder)) = tracker.track(pose) {
                    self.check_and_drive(Physical {
                        speed: f32::from_bits(self.tracking_speed.load(Relaxed)) * k,
                        rudder,
                    })
                    .await;
                }
                *context = tracker.context;
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
        else if let Some(collision) = {
            if let Some(tr) = self.chassis.predict().await {
                self.lidar.check(tr).await
            } else {
                None
            }
        } {
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
            })
        };
    }

    pub(crate) use {join_async, send_async};
}

#[inline]
pub(crate) const fn vector(x: f32, y: f32) -> Vector2<f32> {
    use parry2d::na::{ArrayStorage, Vector};
    Vector::from_array_storage(ArrayStorage([[x, y]]))
}

#[inline]
pub(crate) const fn point(x: f32, y: f32) -> Point2<f32> {
    Point {
        coords: vector(x, y),
    }
}
