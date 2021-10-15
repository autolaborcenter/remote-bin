﻿use async_std::{channel, task};
use pm1_sdk::model::Physical;
use remote_bin::*;
use std::{net::Ipv4Addr, thread};

fn main() {
    let (to_chassis, for_chassis) = channel::unbounded();
    let (to_lidar, for_lidar) = channel::unbounded();
    let (to_follower, for_follower) = channel::unbounded();
    let rtk = false;

    {
        // 监控底盘
        let to_lidar = to_lidar.clone();
        let to_follower = to_follower.clone();
        thread::spawn(move || chassis::supervisor(to_lidar, to_follower, for_chassis));
        // 监控雷达
        let to_chassis = to_chassis.clone();
        thread::spawn(move || lidar::supervisor(to_chassis, for_lidar));
    }
    if rtk {
        // 监控位导
        let to_follower = to_follower.clone();
        thread::spawn(move || rtk::supervisor(to_follower));
        // 执行导航任务
        let to_chassis = to_chassis.clone();
        task::spawn(async move { tracker::task(to_chassis, for_follower).await });
    }

    let mut line = String::new();
    loop {
        line.clear();
        match std::io::stdin().read_line(&mut line) {
            Ok(_) => {
                let tokens = line.split_whitespace().collect::<Vec<_>>();
                match tokens.get(0) {
                    Some(&"show") => {
                        if tokens.len() == 1 {
                            let _ = to_chassis.send(MsgToChassis::PrintStatus);
                        }
                    }
                    Some(&"move") => {
                        if tokens.len() == 3 {
                            if let Some(p) = parse(tokens[1], tokens[2]) {
                                let _ = to_chassis.send(if p.speed == 0.0 {
                                    println!("! 127");
                                    MsgToChassis::Move(p)
                                } else {
                                    MsgToChassis::Predict(Commander::Artificial, p)
                                });
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