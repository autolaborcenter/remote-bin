use super::{
    MsgToChassis::{self, *},
    MsgToLidar,
};
use driver::{Driver, SupersivorEventForSingle::*, SupervisorForSingle};
use pm1_sdk::{PM1Status, PM1};
use std::{
    sync::mpsc::{Receiver, Sender},
    thread,
    time::{Duration, Instant},
};

pub(super) fn supervisor(lidar: Sender<MsgToLidar>, mail_box: Receiver<MsgToChassis>) {
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
            Event(pm1, _) => {
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
                        Predict(p) => {
                            let (model, mut predictor) = pm1.predict();
                            predictor.set_target(p);
                            let _ = lidar.send(MsgToLidar::Check(model, predictor));
                        }
                        Move(p) => pm1.send((Instant::now(), p)),
                    }
                }
            }
        };
        true
    });
}
