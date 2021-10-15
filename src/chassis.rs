use crate::MsgToFollower;

use super::{
    Commander::*,
    MsgToChassis::{self, *},
    MsgToLidar,
};
use pm1_sdk::{
    driver::{Driver, SupersivorEventForSingle::*, SupervisorForSingle},
    PM1Event, PM1Status, PM1,
};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread,
    time::{Duration, Instant},
};

const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(200);

pub fn supervisor(
    to_lidar: Sender<MsgToLidar>,
    to_follower: Sender<MsgToFollower>,
    mail_box: Receiver<MsgToChassis>,
) {
    let mut artificial_deadline = Instant::now();
    SupervisorForSingle::<PM1>::new().join(|e| {
        match e {
            Connected(_, driver) => eprintln!("Connected: {}", driver.status()),
            ConnectFailed => {
                eprintln!("Failed.");
                thread::sleep(Duration::from_secs(1));
            }
            Disconnected => {
                eprintln!("Disconnected.");
                thread::sleep(Duration::from_secs(1));
            }

            Event(pm1, e) => {
                if let Some((time, PM1Event::Odometry(o))) = e {
                    let _ = to_follower.send(MsgToFollower::Relative(time, o.pose));
                }
                while let Ok(msg) = mail_box.try_recv() {
                    match msg {
                        PrintStatus => {
                            let PM1Status {
                                battery_percent,
                                power_switch: _,
                                physical,
                                odometry,
                            } = pm1.status();
                            println!(
                                "S {} {} {} {}",
                                battery_percent, physical.speed, physical.rudder, odometry.s
                            );
                        }
                        Predict(c, p) => {
                            let now = Instant::now();
                            if match c {
                                Artificial => {
                                    artificial_deadline = now + ARTIFICIAL_TIMEOUT;
                                    true
                                }
                                Automatic => now > artificial_deadline,
                            } {
                                let (model, mut predictor) = pm1.predict();
                                predictor.set_target(p);
                                let _ = to_lidar.send(MsgToLidar::Check(model, predictor));
                            }
                        }
                        Move(p) => pm1.send((Instant::now(), p)),
                    }
                }
            }
        };
        true
    });
}
