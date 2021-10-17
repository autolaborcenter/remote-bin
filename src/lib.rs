use async_std::{
    channel::{unbounded, Receiver, Sender},
    task,
};
use parry2d::na::Isometry2;
use std::time::Duration;

mod chassis;
mod lidar;
mod rtk;
mod tracker;

type Trajectory = Box<dyn Iterator<Item = (Duration, Odometry)> + Send>;

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

pub fn launch(rtk: bool) -> (Sender<Command>, Receiver<Event>) {
    let rtk = if rtk {
        rtk::supervisor()
    } else {
        unbounded().1
    };
    let (to_chassis, chassis) = chassis::supervisor();
    let (to_lidar, lidar) = lidar::supervisor();

    task::spawn(async move {
        while let Ok(e) = rtk.recv().await {
            use rtk::Event::*;
            match e {
                Connected => {}
                Disconnected => {}
                SolutionUpdated(t, s) => {}
            }
        }
    });
    task::spawn(async move {
        while let Ok(e) = chassis.recv().await {
            use chassis::Event::*;
            match e {
                Connected => {}
                Disconnected => {}
                StatusUpdated(s) => {}
                OdometryUpdated(t, o) => {}
            }
        }
    });
    task::spawn(async move {
        while let Ok(e) = lidar.recv().await {
            use lidar::Event::*;
            match e {
                Connected => {}
                Disconnected => {}
                FrameEncoded(buf) => {}
            }
        }
    });

    todo!()
}

mod macros {
    macro_rules! send_anyway {
        ($msg:expr => $sender:expr) => {
            let _ = async_std::task::block_on(async { $sender.send($msg).await });
        };
    }

    pub(crate) use send_anyway;
}
