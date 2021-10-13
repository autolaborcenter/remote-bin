use pm1_control_model::{ChassisModel, Physical, Predictor};
use std::{
    net::Ipv4Addr,
    sync::{mpsc::*, Arc, Mutex},
    thread,
};

enum MsgToChassis {
    PrintStatus,
    Predict(Physical),
    Move(Physical),
}

enum MsgToLidar {
    Check(ChassisModel, Predictor),
    Send(Option<Ipv4Addr>),
}

mod chassis;
mod lidar;

fn main() {
    let (to_chassis, for_chassis) = channel();
    let (to_lidar, for_lidar) = channel();

    {
        let to_lidar = to_lidar.clone();
        thread::spawn(move || chassis::supervisor(to_lidar, for_chassis));
    }

    {
        let to_chassis = to_chassis.clone();
        thread::spawn(move || lidar::supervisor(to_chassis, for_lidar));
    }

    let mut line = String::new();
    loop {
        line.clear();
        match std::io::stdin().read_line(&mut line) {
            Ok(_) => {
                let tokens = line.split_whitespace().collect::<Vec<_>>();
                match tokens.get(0) {
                    Some(&"S") => {
                        if tokens.len() == 1 {
                            let _ = to_chassis.send(MsgToChassis::PrintStatus);
                        }
                    }
                    Some(&"T") => {
                        if tokens.len() == 3 {
                            if let Some(p) = parse(tokens[1], tokens[2]) {
                                let _ = to_chassis.send(MsgToChassis::Predict(p));
                            }
                        }
                    }
                    Some(&"P") => {
                        if tokens.len() == 3 {
                            if let Some(p) = parse(tokens[1], tokens[2]) {
                                let _ = to_chassis.send(MsgToChassis::Move(p));
                            }
                        }
                    }
                    Some(&"user") => match tokens.len() {
                        1 => {
                            let _ = to_lidar.send(MsgToLidar::Send(None));
                        }
                        2 => {
                            if let Ok(a) = tokens[1].parse::<Ipv4Addr>() {
                                let _ = to_lidar.send(MsgToLidar::Send(Some(a)));
                            }
                        }
                        _ => {}
                    },
                    _ => {}
                }
            }
            Err(_) => return,
        }
    }
}

fn parse(speed: &str, rudder: &str) -> Option<Physical> {
    let speed = speed.parse();
    let rudder = rudder.parse();
    if speed.is_ok() {
        if rudder.is_ok() {
            Some(Physical {
                speed: speed.unwrap(),
                rudder: rudder.unwrap(),
            })
        } else if speed.unwrap() == 0.0 {
            Some(Physical::RELEASED)
        } else {
            None
        }
    } else {
        None
    }
}
