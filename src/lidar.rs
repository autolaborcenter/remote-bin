use super::{join_async, send_async, CollisionInfo, Odometry, Trajectory};
use async_std::{
    channel::{unbounded, Receiver},
    sync::{Arc, Mutex},
    task,
};
use lidar_faselase::{
    driver::{Driver, Indexer, SupervisorEventForMultiple::*, SupervisorForMultiple},
    FrameCollector, Point, D10,
};
use std::{
    f64::consts::PI,
    sync::atomic::{AtomicBool, Ordering},
    time::{Duration, Instant},
};

mod collision;

#[derive(Clone)]
pub(super) struct Lidar(Arc<Inner>);

pub(super) enum Event {
    Connected,
    Disconnected,
    FrameEncoded(Vec<u8>),
}

struct Inner {
    is_available: AtomicBool,
    frame: Mutex<[FrameCollector; 2]>,
}

impl Lidar {
    #[inline]
    pub async fn check(&self, trajectory: Trajectory) -> Option<CollisionInfo> {
        if self.0.is_available.load(Ordering::Relaxed) {
            collision::detect(self.0.frame.lock().await.as_ref(), trajectory)
        } else {
            None
        }
    }

    pub fn supervisor() -> (Self, Receiver<Event>) {
        let (event, to_extern) = unbounded();
        let lidar_clone = Self(Arc::new(Inner {
            is_available: AtomicBool::new(false),
            frame: Mutex::new([
                FrameCollector {
                    trans: (-141, 0, PI),
                    ..FrameCollector::new()
                },
                FrameCollector {
                    trans: (118, 0, 0.0),
                    ..FrameCollector::new()
                },
            ]),
        }));
        let lidar = lidar_clone.clone();
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
            let mut send_time = Instant::now() + Duration::from_millis(100);

            SupervisorForMultiple::<D10>::new().join(2, |e| {
                match e {
                    Connected(k, driver) => {
                        task::block_on(send_async!(Event::Connected => event));
                        if let Some(i) = indexer.add(k.clone()) {
                            if i == 1 {
                                // 1 号雷达就位，可以开始工作
                                lidar.0.is_available.store(true, Ordering::Relaxed);
                            }
                            // 为雷达设置过滤器
                            driver.send(FILTERS[i]);
                            need_update[i] = false;
                            // 后面的雷达需要重设过滤器
                            for n in &mut need_update[i + 1..] {
                                *n = true;
                            }
                            // 挪动的雷达清除缓存
                            task::block_on(async {
                                for c in &mut lidar.0.frame.lock().await[i..] {
                                    c.clear();
                                }
                            });
                        }
                    }
                    Disconnected(k) => {
                        task::block_on(send_async!(Event::Disconnected => event));
                        if let Some(i) = indexer.remove(k) {
                            // 任何有序号的雷达移除，停止工作
                            lidar.0.is_available.store(false, Ordering::Relaxed);
                            // 前移到当前位置的雷达重设过滤器
                            for n in &mut need_update[i..] {
                                *n = true;
                            }
                            task::block_on(async {
                                for c in &mut lidar.0.frame.lock().await[i..] {
                                    c.clear();
                                }
                            });
                        }
                    }
                    ConnectFailed {
                        current: _,
                        target: _,
                        next_try,
                    } => *next_try = Instant::now() + Duration::from_secs(1),
                    Event(k, e, s) => {
                        let now = Instant::now();
                        // 更新
                        if let Some(j) = indexer.find(&k) {
                            if need_update[j] {
                                let _ = s.send(FILTERS[j]);
                                need_update[j] = false;
                            }
                            if let Some((_, (i, s))) = e {
                                join_async!(async {
                                    lidar.0.frame.lock().await[j].put(i as usize, s)
                                });
                            }
                        }
                        // 发送
                        if indexer.is_full() && now >= send_time {
                            send_time = now + Duration::from_millis(100);
                            let mut buf = vec![255];
                            join_async!(async {
                                let frame = lidar.0.frame.lock().await;
                                frame[1].write_to(&mut buf);
                                frame[0].write_to(&mut buf);
                                let _ = event.send(Event::FrameEncoded(buf)).await;
                            });
                        }
                    }
                }
                2
            });
        });
        (lidar_clone, to_extern)
    }
}
