use pm1_control_model::Physical;
use std::{
    net::Ipv4Addr,
    sync::{mpsc::*, Arc, Mutex},
    thread,
};

enum Request {
    S,
    P(Physical),
    T(Physical),
}

mod chassis;
mod lidar;

fn main() {
    let (sender, receiver) = channel();
    thread::spawn(move || chassis::supervisor(receiver));

    let user_address = Arc::new(Mutex::new(None));
    {
        let user_address = user_address.clone();
        thread::spawn(move || lidar::supervisor(user_address));
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
                            let _ = sender.send(Request::S);
                        }
                    }
                    Some(&"T") => {
                        if tokens.len() == 3 {
                            if let Some(p) = parse(tokens[1], tokens[2]) {
                                let _ = sender.send(Request::T(p));
                            }
                        }
                    }
                    Some(&"P") => {
                        if tokens.len() == 3 {
                            if let Some(p) = parse(tokens[1], tokens[2]) {
                                let _ = sender.send(Request::P(p));
                            }
                        }
                    }
                    Some(&"user") => {
                        if tokens.len() == 2 {
                            if let Ok(a) = tokens[1].parse::<Ipv4Addr>() {
                                *user_address.lock().unwrap() = Some(a);
                            }
                        }
                    }
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
