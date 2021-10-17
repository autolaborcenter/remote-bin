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

struct CollisionInfo(pub Duration, pub Odometry, pub f32);

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
    let (chassis, from_chassis) = chassis::supervisor();
    let (lidar, from_lidar) = lidar::supervisor();
    let (for_extern, command) = unbounded();
    let (event, to_extern) = unbounded();

    {
        let event = event.clone();
        task::spawn(async move {
            while let Ok(e) = command.recv().await {
                use Command::*;
                match e {
                    Move(p) => {
                        if p.speed == 0.0 {
                            let _ = chassis.drive(p).await;
                        } else {
                            if let Some(tr) = chassis.predict(p).await {
                                if let Some(ci) = lidar.check(tr).await {
                                    match ci {
                                        Some(ci) => {
                                            let _ = event.send(Event::CollisionDetected(
                                                chassis.avoid_collision(p, ci).await,
                                            ));
                                        }
                                        None => chassis.drive(p).await,
                                    }
                                }
                            }
                        }
                    }
                    Track(name) => {}
                    Record(name) => {}
                    Pause(bool) => {}
                    Stop => {}
                }
            }
        });
    }

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
    {
        let event = event.clone();
        task::spawn(async move {
            while let Ok(e) = from_chassis.recv().await {
                use chassis::Event::*;
                match e {
                    Connected => {}
                    Disconnected => {}
                    StatusUpdated(s) => {
                        let _ = event.send(Event::ChassisStatusUpdated(s)).await;
                    }
                    OdometryUpdated(t, o) => {
                        let _ = event.send(Event::ChassisOdometerUpdated(o.s, o.a)).await;
                    }
                }
            }
        });
    }
    {
        let event = event.clone();
        task::spawn(async move {
            while let Ok(e) = from_lidar.recv().await {
                use lidar::Event::*;
                match e {
                    Connected => {}
                    Disconnected => {}
                    FrameEncoded(buf) => {
                        let _ = event.send(Event::LidarFrameEncoded(buf)).await;
                    }
                }
            }
        });
    }

    (for_extern, to_extern)
}

use macros::*;
mod macros {
    macro_rules! send_async {
        ($msg:expr => $sender:expr) => {
            let _ = async_std::task::block_on(async { $sender.send($msg).await });
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
