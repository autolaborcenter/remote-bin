use super::{send_async, CollisionInfo, Pose, Trajectory};
use crate::Point;
use async_std::{
    channel::{unbounded, Receiver},
    task,
};
use std::{
    f32::consts::PI,
    time::{Duration, Instant},
};

mod group;

use group::Group;
use lidar::{
    driver::{Indexer, SupervisorEventForMultiple::*, SupervisorForMultiple},
    CONFIG,
};
use m::*;

#[cfg(feature = "faselase")]
mod m {
    pub(super) use lidar_faselase as lidar;
    pub(super) type Device = lidar::Lidar<lidar::D10>;
}

#[cfg(feature = "ld19")]
mod m {
    pub(super) use lidar_ld19 as lidar;
    pub(super) type Device = lidar::Lidar<lidar::LD19>;
}

#[derive(Clone)]
pub(super) struct Lidar(Group);

pub(super) enum Event {
    Connected,
    Disconnected,
    FrameEncoded(Vec<u8>),
}

pub(super) const FILTERS: [fn(Point) -> bool; 2] = [
    |Point { len: _, dir }| {
        const DEG180: u16 = CONFIG.dir_round / 2;
        const DEG90: u16 = DEG180 / 2;
        const DEG30: u16 = DEG90 / 3;
        (DEG90 < dir && dir <= DEG180 - DEG30) || (DEG180 + DEG30 < dir && dir <= (DEG180 + DEG90))
    },
    |Point { len: _, dir }| {
        const LIMIT: u16 = (CONFIG.dir_round as f32 * 1.5 / (2.0 * PI)) as u16;
        dir < LIMIT || (CONFIG.dir_round - LIMIT) <= dir
    },
];

impl Lidar {
    #[inline]
    pub async fn check(&self, trajectory: Trajectory) -> Option<CollisionInfo> {
        self.0.detect(trajectory).await
    }

    pub fn supervisor() -> (Self, Receiver<Event>) {
        let (event, to_extern) = unbounded();
        let (group, mut collectors) = Group::build(&[
            Pose {
                x: -0.141,
                y: 0.0,
                theta: 0.0,
            },
            Pose {
                x: 0.118,
                y: 0.0,
                theta: 0.0,
            },
        ]);
        task::spawn_blocking(move || {
            let mut indexer = Indexer::new(2);
            let mut send_time = Instant::now() + Duration::from_millis(100);

            SupervisorForMultiple::<Device>::new().join(2, |e| {
                match e {
                    Connected(k, driver) => {
                        task::block_on(send_async!(Event::Connected => event));
                        if let Some(i) = indexer.add(k.clone()) {
                            // 为雷达设置过滤器
                            driver.filter = FILTERS[i];
                            // 挪动的雷达清除缓存
                            task::block_on(collectors[i].clear());
                        }
                    }
                    Disconnected(k) => {
                        task::block_on(send_async!(Event::Disconnected => event));
                        // 挪动的雷达清除缓存
                        if let Some(i) = indexer.remove(&k) {
                            task::block_on(collectors[i].clear());
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
                            // 挪动的雷达更新过滤器并清除缓存
                            if indexer.update(j) {
                                let _ = s.send(FILTERS[j]);
                                task::block_on(collectors[j].clear());
                            }
                            if let Some((_, (i, s))) = e {
                                task::block_on(collectors[j].put(i as usize, s))
                            }
                        }
                        // 发送
                        if indexer.len() > 0 && now >= send_time {
                            send_time = now + Duration::from_millis(100);
                            let mut buf = vec![0, 0];
                            collectors[1].write_to(&mut buf);
                            collectors[0].write_to(&mut buf);
                            task::block_on(send_async!(Event::FrameEncoded(buf) => event));
                        }
                    }
                }
                2
            });
        });
        (Self(group), to_extern)
    }
}
