use super::{
    MsgToChassis,
    MsgToLidar::{self, *},
};
use lidar_faselase::{
    driver::{Driver, Indexer, SupervisorEventForMultiple::*, SupervisorForMultiple},
    FrameCollector, Point, D10,
};
use pm1_sdk::model::{Odometry, Physical};
use std::{
    f64::consts::PI,
    net::{IpAddr, SocketAddr, UdpSocket},
    sync::mpsc::{Receiver, Sender},
    thread,
    time::{Duration, Instant},
};

mod collision;

pub(super) fn supervisor(chassis: Sender<MsgToChassis>, mail_box: Receiver<MsgToLidar>) {
    let mut indexer = Indexer::new(2);
    let mut frame = [
        FrameCollector {
            trans: (118, 0, 0.0),
            ..FrameCollector::new()
        },
        FrameCollector {
            trans: (-141, 0, PI),
            ..FrameCollector::new()
        },
    ];
    const FILTERS: [fn(Point) -> bool; 2] = [
        |Point { len: _, dir }| {
            const LIMIT: u16 = 1375; // 5760 * 1.5 / 2π
            dir < LIMIT || (5760 - LIMIT) <= dir
        },
        |Point { len: _, dir }| {
            const DEG90: u16 = 5760 / 4;
            const DEG30: u16 = DEG90 / 3;
            (DEG30 < dir && dir <= DEG90) || ((5760 - DEG90) < dir && dir <= (5760 - DEG30))
        },
    ];

    let socket = UdpSocket::bind("0.0.0.0:0").unwrap();
    let mut address = None;
    let mut send_time = Instant::now() + Duration::from_millis(100);
    let mut update_filter = false;

    SupervisorForMultiple::<D10>::new().join(2, |e| {
        match e {
            Connected(k, lidar) => {
                eprintln!("connected: COM{}", k);
                if let Some(i) = indexer.add(k.clone()) {
                    lidar.send(FILTERS[i]);
                    if i == 0 {
                        update_filter = true;
                    }
                    for c in &mut frame[i..] {
                        c.clear();
                    }
                }
            }
            Disconnected(k) => {
                eprintln!("disconnected: COM{}", k);
                if let Some(i) = indexer.remove(k) {
                    if i == 0 {
                        update_filter = true;
                    }
                    for c in &mut frame[i..] {
                        c.clear();
                    }
                }
            }
            Event(k, e, s) => {
                let now = Instant::now();
                // 更新
                if let Some(j) = indexer.find(&k) {
                    if j == 1 && update_filter {
                        update_filter = false;
                        let _ = s.send(FILTERS[1]);
                    }
                    if let Some((_, (i, s))) = e {
                        frame[j].put(i as usize, s);
                    }
                }
                // 响应请求
                while let Ok(msg) = mail_box.try_recv() {
                    match msg {
                        Check(model, predictor) => {
                            const PERIOD: Duration = Duration::from_millis(40);
                            const PERIOD_SEC: f32 = 0.04;

                            let target = predictor.target;
                            let mut pose = Odometry::ZERO;
                            let mut time = Duration::ZERO;
                            let mut size = 1.0;
                            for status in predictor {
                                time += PERIOD;
                                if time > Duration::from_secs(2) {
                                    let _ = chassis.send(MsgToChassis::Move(target));
                                    break;
                                }
                                let delta = model.physical_to_odometry(Physical {
                                    speed: status.speed * PERIOD_SEC,
                                    ..status
                                });
                                size += delta.s;
                                pose += delta;
                                if collision::detect(&frame, pose.pose, size) {
                                    if time > Duration::from_millis(300) {
                                        let k = time.as_secs_f32() / 2.0;
                                        let _ = chassis.send(MsgToChassis::Move(Physical {
                                            speed: target.speed * k,
                                            ..target
                                        }));
                                    }
                                    break;
                                }
                            }
                        }
                        Send(a) => address = a.map(|a| SocketAddr::new(IpAddr::V4(a), 5005)),
                    }
                }
                // 发送
                if let Some(a) = address {
                    if now >= send_time {
                        send_time = now + Duration::from_millis(100);
                        let mut buf = Vec::new();
                        frame[0].write_to(&mut buf);
                        frame[1].write_to(&mut buf);
                        eprintln!("{:?}", socket.send_to(buf.as_slice(), a));
                    }
                }
            }
            ConnectFailed { current, target } => {
                eprintln!("{}/{}", current, target);
                thread::sleep(Duration::from_secs(1));
            }
        }
        2
    });
}
