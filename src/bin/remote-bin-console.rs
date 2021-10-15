use async_std::{
    net::UdpSocket,
    sync::{Arc, Mutex},
    task,
};
use remote_bin::*;
use std::net::SocketAddr;

#[async_std::main]
async fn main() {
    let (command, event) = launch(false);
    let address = Arc::new(Mutex::new(None));
    let status = Arc::new(Mutex::new((
        PM1Status {
            battery_percent: 0,
            power_switch: false,
            physical: Physical::ZERO,
        },
        Odometry::ZERO,
        0f32,
    )));
    {
        let address = address.clone();
        let status = status.clone();
        task::spawn(async move {
            let socket = UdpSocket::bind("0.0.0.0:0").await.unwrap();
            while let Ok(e) = event.recv().await {
                use Event::*;
                match e {
                    ChassisStatusUpdated(e) => match e {
                        PM1Event::Battery(b) => status.lock().await.0.battery_percent = b,
                        PM1Event::PowerSwitch(b) => status.lock().await.0.power_switch = b,
                        PM1Event::Physical(p) => status.lock().await.0.physical = p,
                        PM1Event::Odometry(delta) => status.lock().await.1 += delta,
                    },
                    LidarFrame(buf) => {
                        if let Some(a) = *address.lock().await {
                            let _ = socket.send_to(buf.as_slice(), a).await;
                        }
                    }
                    DetectCollision(risk) => {
                        status.lock().await.2 = risk;
                    }
                }
            }
        });
    }
    loop {
        let mut line = String::new();
        loop {
            line.clear();
            match async_std::io::stdin().read_line(&mut line).await {
                Ok(_) => {
                    let tokens = line.split_whitespace().collect::<Vec<_>>();
                    match tokens.get(0) {
                        Some(&"show") => {
                            if tokens.len() == 1 {
                                let (
                                    PM1Status {
                                        battery_percent,
                                        power_switch: _,
                                        physical: Physical { speed, rudder },
                                    },
                                    Odometry { s, a: _, pose: _ },
                                    risk,
                                ) = *status.lock().await;
                                println!(
                                    "S {} {} {} {} \n ! {}",
                                    battery_percent, speed, rudder, s, risk
                                );
                            }
                        }
                        Some(&"move") => {
                            if tokens.len() == 3 {
                                if let Some(p) = parse(tokens[1], tokens[2]) {
                                    let _ = command.send(Command::Move(p));
                                }
                            }
                        }
                        Some(&"user") => match tokens.len() {
                            1 => {
                                *address.lock().await = None;
                            }
                            2 => {
                                if let Ok(a) = tokens[1].parse::<SocketAddr>() {
                                    *address.lock().await = Some(a);
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
