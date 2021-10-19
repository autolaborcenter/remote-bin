use super::{call, send_async, CollisionInfo, Odometry, Trajectory};
use async_std::{
    channel::{unbounded, Receiver, Sender},
    sync::Arc,
    task,
};
use lidar_faselase::{
    driver::{Driver, Indexer, SupervisorEventForMultiple::*, SupervisorForMultiple},
    FrameCollector, Point, D10,
};
use std::{
    f64::consts::PI,
    io::Write,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

mod collision;

#[derive(Clone)]
pub(super) struct Lidar(Sender<Command>, Arc<AtomicBool>);

impl Lidar {
    #[inline]
    pub async fn check(&self, trajectory: Trajectory) -> Option<Option<CollisionInfo>> {
        if self.1.load(Ordering::Relaxed) {
            // 如果可以工作
            call!(self.0; Command, trajectory)
        } else {
            // 否则直接放行
            Some(None)
        }
    }
}

struct Command(Trajectory, Sender<Option<CollisionInfo>>);

pub(super) enum Event {
    Connected,
    Disconnected,
    FrameEncoded(Vec<u8>),
}

pub(super) fn supervisor() -> (Lidar, Receiver<Event>) {
    let (for_extern, command) = unbounded();
    let (event, to_extern) = unbounded();
    let is_available_clone = Arc::new(AtomicBool::new(false));
    let is_available = is_available_clone.clone();
    task::spawn_blocking(move || {
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

        let mut indexer = Indexer::new(2);
        let mut need_update = [false, false];
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
        let mut send_time = Instant::now() + Duration::from_millis(100);

        SupervisorForMultiple::<D10>::new().join(2, |e| {
            match e {
                Connected(k, lidar) => {
                    task::block_on(send_async!(Event::Connected => event));
                    if let Some(i) = indexer.add(k.clone()) {
                        if i == 1 {
                            // 1 号雷达就位，可以开始工作
                            is_available.store(true, Ordering::Relaxed);
                        }
                        // 为雷达设置过滤器
                        lidar.send(FILTERS[i]);
                        need_update[i] = false;
                        // 后面的雷达需要重设过滤器
                        for n in &mut need_update[i + 1..] {
                            *n = true;
                        }
                        // 挪动的雷达清除缓存
                        for c in &mut frame[i..] {
                            c.clear();
                        }
                    }
                }
                Disconnected(k) => {
                    task::block_on(send_async!(Event::Disconnected => event));
                    if let Some(i) = indexer.remove(k) {
                        // 任何有序号的雷达移除，停止工作
                        is_available.store(false, Ordering::Relaxed);
                        // 前移到当前位置的雷达重设过滤器
                        for n in &mut need_update[i..] {
                            *n = true;
                        }
                        for c in &mut frame[i..] {
                            c.clear();
                        }
                    }
                }
                ConnectFailed {
                    current: _,
                    target: _,
                    next_try,
                } => {
                    *next_try = Instant::now() + Duration::from_secs(1);
                    // 放行所有等待期间到来的询问
                    while let Ok(Command(_, r)) = command.try_recv() {
                        task::block_on(send_async!(None => r));
                    }
                }
                Event(k, e, s) => {
                    let now = Instant::now();
                    // 更新
                    if let Some(j) = indexer.find(&k) {
                        if need_update[j] {
                            let _ = s.send(FILTERS[j]);
                            need_update[j] = false;
                        }
                        if let Some((_, (i, s))) = e {
                            frame[j].put(i as usize, s);
                        }
                    }
                    if indexer.is_full() {
                        // 响应请求
                        let mut trajectory = None;
                        while let Ok(tr) = command.try_recv() {
                            trajectory = Some(tr);
                        }
                        if let Some(Command(tr, r)) = trajectory {
                            task::block_on(send_async!(collision::detect(&frame, tr) => r));
                        }
                        // 发送
                        if now >= send_time {
                            send_time = now + Duration::from_millis(100);
                            let mut buf = Vec::new();
                            let _ = buf.write_all(&[255]);
                            frame[1].write_to(&mut buf);
                            frame[0].write_to(&mut buf);
                            task::block_on(send_async!(Event::FrameEncoded(buf) => event));
                        }
                    } else {
                        // 放行所有等待期间到来的询问
                        while let Ok(Command(_, r)) = command.try_recv() {
                            task::block_on(send_async!(None => r));
                        }
                    }
                }
            }
            2
        });
    });
    (Lidar(for_extern, is_available_clone), to_extern)
}
