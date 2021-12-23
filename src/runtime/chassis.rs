use super::{join_async, send_async, Physical, Trajectory};
use async_std::{
    channel::{unbounded, Receiver},
    sync::{Arc, Mutex},
    task,
};
use pm1_sdk::{
    driver::{SupervisorEventForSingle::*, SupervisorForSingle},
    model::{Pm1Model, Wheels},
    PM1Event, PM1Status, PM1,
};
use std::{
    sync::atomic::{AtomicU64, Ordering::Relaxed},
    time::{Duration, Instant},
};

#[derive(Clone)]
pub(super) struct Chassis(Arc<Inner>);

pub(super) enum Event {
    Connected,
    Disconnected,
    StatusUpdated(PM1Status),
    WheelsUpdated(Instant, Wheels),
}

struct Inner {
    raw_target: AtomicU64,
    target: Mutex<(Instant, Physical)>,
    model: Mutex<Option<Pm1Model>>,
    predictor: Mutex<Option<Trajectory>>,
}

impl Chassis {
    #[inline]
    pub async fn drive(&self, p: Physical) {
        let now = Instant::now();
        *self.0.target.lock().await = (now, p);
    }

    #[inline]
    pub async fn update_model(&self, m: Pm1Model) {
        *self.0.model.lock().await = Some(m);
    }

    #[inline]
    pub async fn store_raw_target(&self, p: Physical) {
        self.0
            .raw_target
            .store(unsafe { *(&p as *const _ as *const _) }, Relaxed);
    }

    #[inline]
    pub async fn predict(&self) -> Option<Trajectory> {
        self.0.predictor.lock().await.clone().map(|mut pre| {
            pre.predictor.target =
                unsafe { *(&self.0.raw_target.load(Relaxed) as *const _ as *const _) };
            pre
        })
    }

    pub fn supervisor() -> (Self, Receiver<Event>) {
        let (event, to_extern) = unbounded();
        let now = Instant::now();
        let chassis_clone = Self(Arc::new(Inner {
            raw_target: AtomicU64::new(unsafe { *(&Physical::RELEASED as *const _ as *const _) }),
            target: Mutex::new((now, Physical::RELEASED)),
            model: Default::default(),
            predictor: Default::default(),
        }));
        let chassis = chassis_clone.clone();
        task::spawn_blocking(move || {
            SupervisorForSingle::<PM1>::default().join(|e| {
                match e {
                    Connected(_, driver) => {
                        join_async!(
                            send_async!(Event::Connected => event),
                            send_async!(Event::StatusUpdated(*driver.status()) => event),
                            chassis.set_predictor(driver),
                            chassis.0.model.lock(),
                        )
                        .3
                        .take()
                        .map(|m| driver.model = m);
                    }
                    Disconnected => {
                        join_async! {
                            send_async!(Event::Disconnected => event),
                            async { *chassis.0.predictor.lock().await = None },
                        };
                    }
                    ConnectFailed => {
                        task::block_on(task::sleep(Duration::from_secs(1)));
                    }
                    Event(driver, e) => {
                        driver.set_target(*task::block_on(chassis.0.target.lock()));
                        task::block_on(chassis.0.model.lock())
                            .take()
                            .map(|m| driver.model = m);
                        use PM1Event::*;
                        match e {
                            Some((time, Wheels(wheels))) => {
                                task::block_on(
                                    send_async!(Event::WheelsUpdated(time, wheels) => event),
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
        *self.0.predictor.lock().await = Some(Box::new(driver.trajectory_predictor()));
    }

    #[inline]
    async fn set_current(&self, p: Physical) {
        if let Some(ref mut pre) = self.0.predictor.lock().await.as_mut() {
            pre.predictor.current = p;
        }
    }
}
