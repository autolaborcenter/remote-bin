use super::{macros::send_anyway, Odometry, Trajectory};
use async_std::{
    channel::{unbounded, Receiver, Sender},
    task,
};
use lidar_faselase::{
    driver::{Driver, Indexer, SupervisorEventForMultiple::*, SupervisorForMultiple},
    FrameCollector, Point, D10,
};
use std::{
    f64::consts::PI,
    io::Write,
    thread,
    time::{Duration, Instant},
};

mod collision;

pub(super) struct CollisionInfo(Duration, Odometry, f32);

pub(super) type Command = (Trajectory, Sender<Option<CollisionInfo>>);

pub(super) enum Event {
    Connected,
    Disconnected,
    FrameEncoded(Vec<u8>),
}

// pub(super) struct Message(pub ChassisModel, pub Predictor);

pub(super) fn supervisor() -> (Sender<Command>, Receiver<Event>) {
    let (for_extern, command) = unbounded::<Command>();
    let (event, to_extern) = unbounded();
    task::spawn_blocking(move || {
        let mut indexer = Indexer::new(2);
        let mut frame = [
            FrameCollector {
                trans: (-141, 0, PI),
                ..FrameCollector::new()
            },
            FrameCollector {
                trans: (118, 0, 0.0),
                ..FrameCollector::new()
            },
        ];
        const FILTERS: [fn(Point) -> bool; 2] = [
            |Point { len: _, dir }| {
                const DEG90: u16 = 5760 / 4;
                const DEG30: u16 = DEG90 / 3;
                (DEG30 < dir && dir <= DEG90) || ((5760 - DEG90) < dir && dir <= (5760 - DEG30))
            },
            |Point { len: _, dir }| {
                const LIMIT: u16 = 1375; // 5760 * 1.5 / 2π
                dir < LIMIT || (5760 - LIMIT) <= dir
            },
        ];

        let mut send_time = Instant::now() + Duration::from_millis(100);
        let mut update_filter = 2;

        SupervisorForMultiple::<D10>::new().join(2, |e| {
            match e {
                Connected(k, lidar) => {
                    send_anyway!(Event::Connected => event);
                    if let Some(i) = indexer.add(k.clone()) {
                        lidar.send(FILTERS[i]);
                        if i == 0 {
                            update_filter = 1;
                        }
                        for c in &mut frame[i..] {
                            c.clear();
                        }
                    }
                }
                Disconnected(k) => {
                    send_anyway!(Event::Disconnected => event);
                    if let Some(i) = indexer.remove(k) {
                        if i == 0 {
                            update_filter = 0;
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
                        if j == update_filter {
                            update_filter = 2;
                            let _ = s.send(FILTERS[j]);
                        }
                        if let Some((_, (i, s))) = e {
                            frame[j].put(i as usize, s);
                        }
                    }
                    // 响应请求
                    let mut trajectory = None;
                    while let Ok(tr) = command.try_recv() {
                        trajectory = Some(tr);
                    }
                    if let Some((tr, r)) = trajectory {
                        send_anyway!(collision::detect(&frame, tr) => r);
                    }
                    // 发送
                    if now >= send_time {
                        send_time = now + Duration::from_millis(100);
                        let mut buf = Vec::new();
                        let _ = buf.write_all(&[255]);
                        frame[1].write_to(&mut buf);
                        frame[0].write_to(&mut buf);
                        send_anyway!(Event::FrameEncoded(buf) => event);
                    }
                }
                ConnectFailed { current, target } => {
                    thread::sleep(Duration::from_secs(1));
                }
            }
            2
        });
    });
    (for_extern, to_extern)
}
