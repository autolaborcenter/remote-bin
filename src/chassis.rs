use super::Request;
use driver::{Driver, SupersivorEventForSingle::*, SupervisorForSingle};
use pm1_control_model::{Odometry, Physical};
use pm1_sdk::{PM1Status, PM1};
use std::{
    sync::mpsc::Receiver,
    thread,
    time::{Duration, Instant},
};

pub(super) fn supervisor(receiver: Receiver<Request>) {
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
            Event(pm1, _) => match receiver.try_recv() {
                Ok(Request::S) => {
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
                Ok(Request::T(p)) => {
                    let mut pose = Odometry::ZERO;
                    let (model, mut predictor) = pm1.predict();
                    predictor.set_target(p);
                    print!("P {}|{}", p.speed, p.rudder);
                    for _ in 0..20 {
                        for _ in 0..5 {
                            match predictor.next() {
                                Some(s) => {
                                    pose += model.physical_to_odometry(Physical {
                                        speed: s.speed * 0.02,
                                        ..s
                                    });
                                }
                                None => {
                                    pose += model.physical_to_odometry(Physical {
                                        speed: p.speed * 0.02,
                                        ..p
                                    });
                                }
                            }
                        }
                        print!(
                            " {},{},{}",
                            (pose.pose.translation.vector[0] * 1000.0) as u32,
                            (pose.pose.translation.vector[1] * 1000.0) as u32,
                            pose.pose.rotation.angle()
                        );
                    }
                    println!();
                }
                Ok(Request::P(p)) => pm1.send((Instant::now(), p)),
                _ => {}
            },
        };
        true
    });
}
