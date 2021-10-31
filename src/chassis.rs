use super::{join_async, send_async, Odometry, Physical, Trajectory};
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
    pub async fn predict(&self, p: Physical) -> Trajectory {
        if let Some(ref pre) = self.0.predictor.lock().await.as_ref() {
            let mut pre = Box::new(pre.clone());
            pre.predictor.current = p;
            pre as Trajectory
        } else {
            Box::new(NonePredictor) as Trajectory
        }
    }

    pub fn supervisor() -> (Self, Receiver<Event>) {
        let (event, to_extern) = unbounded();
        let chassis_clone = Self(Arc::new(Inner {
            target: Mutex::new((Instant::now(), Physical::RELEASED)),
            predictor: Mutex::new(Box::new(None)),
        }));
        let chassis = chassis_clone.clone();
        task::spawn_blocking(move || {
            let mut odometry = Odometry::ZERO;

            SupervisorForSingle::<PM1>::new().join(|e| {
                match e {
                    Connected(_, driver) => {
                        join_async!(
                            chassis.set_predictor(driver),
                            send_async!(Event::Connected => event),
                            send_async!(Event::StatusUpdated(*driver.status()) => event)
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
                        if let Some((time, e)) = e {
                            if let PM1Event::Odometry(delta) = e {
                                odometry += delta;
                                join_async!(
                                    send_async!(Event::OdometryUpdated(time, odometry) => event)
                                );
                            } else {
                                join_async!(
                                    chassis.set_current(driver.status().physical),
                                    send_async!(Event::StatusUpdated(*driver.status()) => event),
                                );
                            }
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
