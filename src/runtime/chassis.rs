﻿use super::{join_async, send_async, Odometry, Physical, Trajectory};
use async_std::{
    channel::{unbounded, Receiver},
    sync::{Arc, Mutex},
    task,
};
use lidar_faselase::driver::Driver;
use pm1_sdk::{
    driver::{SupersivorEventForSingle::*, SupervisorForSingle},
    model::TrajectoryPredictor,
    PM1Event, PM1Status, PM1,
};
use std::{
    sync::atomic::{AtomicU64, Ordering::Relaxed},
    thread,
    time::{Duration, Instant},
};

#[derive(Clone)]
pub(super) struct Chassis(Arc<Inner>);

pub(super) enum Event {
    Connected,
    Disconnected,

    StatusUpdated(PM1Status),
    OdometryUpdated(Instant, Odometry),
}

struct Inner {
    raw_target: AtomicU64,
    target: Mutex<(Instant, Physical)>,
    predictor: Mutex<Box<Option<TrajectoryPredictor>>>,
}

struct NonePredictor;

impl Chassis {
    #[inline]
    pub async fn drive(&self, p: Physical) {
        let now = Instant::now();
        *self.0.target.lock().await = (now, p);
    }

    #[inline]
    pub async fn store_raw_target(&self, p: Physical) {
        self.0.raw_target.store(*u64_of(&p), Relaxed);
    }

    #[inline]
    pub async fn predict(&self) -> Trajectory {
        if let Some(ref pre) = self.0.predictor.lock().await.as_ref() {
            let mut pre = Box::new(pre.clone());
            pre.predictor.target = *physical_of(&self.0.raw_target.load(Relaxed));
            pre as Trajectory
        } else {
            Box::new(NonePredictor) as Trajectory
        }
    }

    pub fn supervisor() -> (Self, Receiver<Event>) {
        let (event, to_extern) = unbounded();
        let now = Instant::now();
        let chassis_clone = Self(Arc::new(Inner {
            raw_target: AtomicU64::new(*u64_of(&Physical::RELEASED)),
            target: Mutex::new((now, Physical::RELEASED)),
            predictor: Mutex::new(Box::new(None)),
        }));
        let chassis = chassis_clone.clone();
        task::spawn_blocking(move || {
            let mut odometry = Odometry::ZERO;

            SupervisorForSingle::<PM1>::new().join(|e| {
                match e {
                    Connected(_, driver) => {
                        let status = driver.status();
                        join_async!(
                            chassis.set_predictor(driver),
                            send_async!(Event::Connected => event),
                            send_async!(Event::StatusUpdated(*status) => event),
                        );
                    }
                    Disconnected => {
                        join_async!(
                            async { *chassis.0.predictor.lock().await = Box::new(None) },
                            send_async!(Event::Disconnected => event)
                        );
                    }
                    ConnectFailed => {
                        thread::sleep(Duration::from_secs(1));
                    }
                    Event(driver, e) => {
                        driver.send(*task::block_on(chassis.0.target.lock()));
                        use PM1Event::*;
                        match e {
                            Some((time, Odometry(delta))) => {
                                odometry += delta;
                                task::block_on(
                                    send_async!(Event::OdometryUpdated(time, odometry) => event),
                                );
                            }
                            Some(_) => {
                                join_async!(
                                    chassis.set_current(driver.status().physical),
                                    send_async!(Event::StatusUpdated(*driver.status()) => event),
                                );
                            }
                            None => {}
                        }
                    }
                };
                true
            });
        });
        (chassis_clone, to_extern)
    }

    #[inline]
    async fn set_predictor(&self, driver: &PM1) {
        *self.0.predictor.lock().await = Box::new(Some(driver.trajectory_predictor()));
    }

    #[inline]
    async fn set_current(&self, p: Physical) {
        if let Some(ref mut pre) = self.0.predictor.lock().await.as_mut() {
            pre.predictor.current = p;
        }
    }
}

impl Iterator for NonePredictor {
    type Item = (Duration, Odometry);

    fn next(&mut self) -> Option<Self::Item> {
        None
    }
}

#[inline]
fn u64_of<'a>(p: &'a Physical) -> &'a u64 {
    unsafe { *(&p as *const _ as *const _) }
}

#[inline]
fn physical_of<'a>(c: &'a u64) -> &'a Physical {
    unsafe { *(&c as *const _ as *const _) }
}