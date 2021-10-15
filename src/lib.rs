use async_std::{
    channel::{unbounded, Receiver, Sender},
    task,
};
use std::{thread, time::Duration};

mod chassis;
mod lidar;
mod rtk;
mod tracker;

pub use pm1_sdk::{
    model::{Odometry, Physical},
    PM1Event, PM1Status,
};

pub enum Command {
    Move(Physical),
}

pub enum Event {
    ChassisStatusUpdated(PM1Event),
    LidarFrame(Vec<u8>),
    DetectCollision(f32),
}

pub fn launch(rtk: bool) -> (Sender<Command>, Receiver<Event>) {
    let (to_chassis, for_chassis) = unbounded();
    let (to_lidar, for_lidar) = unbounded();
    let (to_follower, for_follower) = unbounded();
    let (to_extern, events) = unbounded();
    let (for_extern, command) = unbounded();

    {
        // 监控底盘
        let to_lidar = to_lidar.clone();
        let to_follower = to_follower.clone();
        let to_extern = to_extern.clone();
        thread::spawn(move || chassis::supervisor(to_lidar, to_follower, to_extern, for_chassis));
    }
    {
        // 监控雷达
        let to_chassis = to_chassis.clone();
        let to_extern = to_extern.clone();
        thread::spawn(move || lidar::supervisor(to_chassis, to_extern, for_lidar));
    }
    if rtk {
        // 监控位导
        let to_follower = to_follower.clone();
        thread::spawn(move || rtk::supervisor(to_follower));
        // 执行导航任务
        let to_chassis = to_chassis.clone();
        task::spawn(async move { tracker::task(to_chassis, for_follower).await });
    }
    {
        task::spawn(async move {
            while let Ok(cmd) = command.recv().await {
                match cmd {
                    Command::Move(p) => {
                        if p.speed == 0.0 {
                            let _ = to_chassis.send(chassis::Message::Move(p)).await;
                            let _ = to_extern.send(Event::DetectCollision(0.0)).await;
                        } else {
                            let _ = to_chassis
                                .send(chassis::Message::PredictArtificial(p))
                                .await;
                        }
                    }
                }
            }
        });
    }

    (for_extern, events)
}

#[macro_export]
macro_rules! send_anyway {
    ($msg:expr => $sender:expr) => {
        let _ = async_std::task::block_on(async { $sender.send($msg).await });
    };
}
