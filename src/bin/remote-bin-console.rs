use async_std::{channel::unbounded, net::UdpSocket, task};
use robot_bin::{Odometry, PM1Status, Physical, Robot};
use std::net::SocketAddr;

enum Event {
    ShowRequest,
    UpdateUser(Option<SocketAddr>),
    Internal(robot_bin::Event),
    Exit,
}

#[async_std::main]
async fn main() {
    let (robot, events) = Robot::spawn(false).await;
    let (s0, receiver) = unbounded();
    let s1 = s0.clone();
    task::spawn(async move {
        while let Ok(e) = events.recv().await {
            let _ = s0.send(Event::Internal(e)).await;
        }
    });
    task::spawn(async move {
        let mut line = String::new();
        loop {
            line.clear();
            match async_std::io::stdin().read_line(&mut line).await {
                Ok(_) => {
                    let tokens = line.split_whitespace().collect::<Vec<_>>();
                    match tokens.get(0) {
                        Some(&"show") => {
                            if tokens.len() == 1 {
                                let _ = s1.send(Event::ShowRequest).await;
                            }
                        }
                        Some(&"move") => {
                            if tokens.len() == 3 {
                                if let Some(p) = parse(tokens[1], tokens[2]) {
                                    robot.drive(p).await;
                                }
                            }
                        }
                        Some(&"user") => match tokens.len() {
                            1 => {
                                let _ = s1.send(Event::UpdateUser(None)).await;
                            }
                            2 => {
                                if let Ok(a) = tokens[1].parse::<SocketAddr>() {
                                    let _ = s1.send(Event::UpdateUser(Some(a))).await;
                                }
                            }
                            _ => {}
                        },
                        _ => {}
                    }
                }
                Err(_) => {
                    let _ = s1.send(Event::Exit).await;
                }
            }
        }
    });

    let socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
    let mut address = None;
    let mut status = PM1Status {
        battery_percent: 0,
        power_switch: false,
        physical: Physical::ZERO,
    };
    let mut odometer = (0.0, 0.0);
    let mut _pose = Odometry::ZERO.pose;
    let mut risk = 0.0;
    loop {
        match receiver.recv().await {
            Ok(Event::ShowRequest) => {
                println!(
                    "S {} {} {} {}\n ! {}",
                    status.battery_percent,
                    status.physical.speed,
                    status.physical.rudder,
                    odometer.0,
                    risk
                );
            }
            Ok(Event::UpdateUser(a)) => address = a,
            Ok(Event::Internal(e)) => {
                use robot_bin::Event::*;
                match e {
                    ChassisStatusUpdated(s) => status = s,
                    ChassisOdometerUpdated(s, a) => odometer = (s, a),
                    PoseUpdated(p) => _pose = p,
                    LidarFrameEncoded(buf) => {
                        if let Some(a) = address {
                            let _ = socket.send_to(buf.as_slice(), a).await;
                        }
                    }
                    CollisionDetected(r) => risk = r,
                }
            }
            Ok(Event::Exit) | Err(_) => break,
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
