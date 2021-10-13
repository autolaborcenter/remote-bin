use crate::{MsgToChassis, MsgToLidar};
use driver::{Driver, Indexer, SupervisorEventForMultiple::*, SupervisorForMultiple};
use lidar_faselase::{FrameCollector, Point, D10};
use std::{
    f64::consts::PI,
    net::{IpAddr, SocketAddr, UdpSocket},
    sync::mpsc::{Receiver, Sender},
    thread,
    time::{Duration, Instant},
};

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
                println!("connected: COM{}", k);
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
                println!("disconnected: COM{}", k);
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
                if let Some(j) = indexer.find(&k) {
                    if j == 1 && update_filter {
                        update_filter = false;
                        let _ = s.send(FILTERS[1]);
                    }
                    if let Some((_, (i, s))) = e {
                        frame[j].put(i as usize, s);
                    }
                }
                while let Ok(msg) = mail_box.try_recv() {
                    match msg {
                        MsgToLidar::Check(model, predictor) => {
                            todo!()
                        }
                        MsgToLidar::Send(a) => {
                            address = a.map(|a| SocketAddr::new(IpAddr::V4(a), 5005))
                        }
                    }
                }
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
