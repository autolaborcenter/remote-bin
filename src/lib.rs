use async_std::{
    channel::{unbounded, Receiver, Sender},
    task,
};
use parry2d::na::Isometry2;
use std::time::Instant;

mod chassis;
mod lidar;
mod rtk;
mod tracker;

pub use pm1_sdk::{
    model::{Odometry, Physical},
    PM1Status,
};

pub enum Command {
    Move(Physical),
    Track(String),
    Record(String),
    Pause(bool),
    Stop,
}

pub enum Event {
    ChassisStatusUpdated(PM1Status),
    ChassisOdometerUpdated(f32, f32),
    PoseUpdated(Isometry2<f32>),
    LidarFrameEncoded(Vec<u8>),
    CollisionDetected(f32),
}

enum Locating {
    Absolute(Instant, Isometry2<f32>),
    Relative(Instant, Isometry2<f32>),
}

pub fn launch(rtk: bool) -> (Sender<Command>, Receiver<Event>) {
    let (to_chassis, for_chassis) = unbounded();
    let (to_lidar, for_lidar) = unbounded();
    let (to_tracker, for_tracker) = unbounded();
    let (to_app, for_app) = unbounded();
    let (to_this, for_this) = unbounded();

    {
        // 监控底盘
        let to_lidar = to_lidar.clone();
        let to_follower = to_tracker.clone();
        let to_extern = to_app.clone();
        task::spawn_blocking(move || {
            chassis::supervisor(to_lidar, to_follower, to_extern, for_chassis)
        });
    }
    {
        // 监控雷达
        let to_chassis = to_chassis.clone();
        let to_extern = to_app.clone();
        task::spawn_blocking(move || lidar::supervisor(to_chassis, to_extern, for_lidar));
    }
    if rtk {
        // 监控位导
        let to_follower = to_tracker.clone();
        task::spawn_blocking(move || rtk::supervisor(to_follower));
    }
    // {
    //     // 导航
    //     let to_chassis = to_chassis.clone();
    //     let to_app = to_app.clone();
    //     task::spawn(async move { tracker::task(to_chassis, to_app, for_tracker).await });
    // }
    {
        // 交互
        task::spawn(async move {
            while let Ok(cmd) = for_this.recv().await {
                use Command::*;
                match cmd {
                    Move(p) => {
                        if p.speed == 0.0 {
                            let _ = to_chassis.send(chassis::Message::Move(p)).await;
                            let _ = to_app.send(Event::CollisionDetected(0.0)).await;
                        } else {
                            let _ = to_chassis
                                .send(chassis::Message::PredictArtificial(p))
                                .await;
                        }
                    }
                    Track(name) => {}
                    Record(String) => {}
                    Pause(bool) => {}
                    Stop => {}
                }
            }
        });
    }

    (to_this, for_app)
}

mod macros {
    macro_rules! send_anyway {
        ($msg:expr => $sender:expr) => {
            let _ = async_std::task::block_on(async { $sender.send($msg).await });
        };
    }

    pub(crate) use send_anyway;
}
