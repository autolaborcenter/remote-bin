use super::{join_async, send_async, Odometry, Physical, Trajectory};
use async_std::{
    channel::{unbounded, Receiver},
    sync::{Arc, Mutex},
    task,
};
use pm1_sdk::{
    driver::{SupersivorEventForSingle::*, SupervisorForSingle},
    model::{ChassisModel, Predictor},
    PM1Event, PM1Status, CONTROL_PERIOD, PM1,
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
    predictor: Mutex<Option<(ChassisModel, Predictor)>>,
}

impl Chassis {
    #[inline]
    pub async fn drive(&self, p: Physical) {
        let now = Instant::now();
        *self.0.target.lock().await = (now, p);
    }

    #[inline]
    pub async fn predict(&self, p: Physical) -> Option<Trajectory> {
        self.0
            .predictor
            .lock()
            .await
            .clone()
            .map(|(model, mut predictor)| {
                predictor.target = p;
                Box::new(TrajectoryPredictor { model, predictor }) as Trajectory
            })
    }

    #[inline]
    async fn set_current(&self, p: Physical) {
        if let Some((_, pre)) = self.0.predictor.lock().await.as_mut() {
            pre.current = p;
        }
    }

    pub fn supervisor() -> (Self, Receiver<Event>) {
        let (event, to_extern) = unbounded();
        let chassis_clone = Self(Arc::new(Inner {
            target: Mutex::new((Instant::now(), Physical::RELEASED)),
            predictor: Mutex::new(None),
        }));
        let chassis = chassis_clone.clone();
        task::spawn_blocking(move || {
            let mut odometry = Odometry::ZERO;

            SupervisorForSingle::<PM1>::new().join(|e| {
                match e {
                    Connected(_, driver) => {
                        join_async!(
                            async { *chassis.0.predictor.lock().await = Some(driver.predict()) },
                            send_async!(Event::Connected => event),
                            send_async!(Event::StatusUpdated(*driver.status()) => event)
                        );
                    }
                    Disconnected => {
                        join_async!(
                            async { *chassis.0.predictor.lock().await = None },
                            send_async!(Event::Disconnected => event)
                        );
                    }
                    ConnectFailed => {
                        thread::sleep(Duration::from_secs(1));
                    }
                    Event(driver, e) => {
                        if let Some((time, e)) = e {
                            if let PM1Event::Odometry(delta) = e {
                                odometry += delta;
                                join_async!(
                                    send_async!(Event::OdometryUpdated(time, odometry) => event)
                                );
                            } else {
                                let status = driver.status();
                                join_async!(
                                    chassis.set_current(status.physical),
                                    send_async!(Event::StatusUpdated(*status) => event),
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
}

struct TrajectoryPredictor {
    model: ChassisModel,
    predictor: Predictor,
}

impl Iterator for TrajectoryPredictor {
    type Item = (Duration, Odometry);

    fn next(&mut self) -> Option<Self::Item> {
        match self.predictor.next() {
            Some(Physical { speed, rudder }) => Some((
                CONTROL_PERIOD,
                self.model.physical_to_odometry(Physical {
                    speed: speed * CONTROL_PERIOD.as_secs_f32(),
                    rudder,
                }),
            )),
            None => None,
        }
    }
}
