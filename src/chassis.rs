use super::{call, send_async, Trajectory};
use async_std::{
    channel::{unbounded, Receiver, Sender},
    sync::Arc,
    task,
};
use futures::join;
use pm1_sdk::{
    driver::{SupersivorEventForSingle::*, SupervisorForSingle},
    model::{ChassisModel, Odometry, Physical, Predictor},
    PM1Event, PM1Status, CONTROL_PERIOD, PM1,
};
use std::{
    sync::atomic::{AtomicBool, Ordering},
    thread,
    time::{Duration, Instant},
};

#[derive(Clone)]
pub(super) struct Chassis(Sender<Command>, Arc<AtomicBool>);

impl Chassis {
    #[inline]
    pub async fn predict(&self, p: Physical) -> Option<Trajectory> {
        if self.1.load(Ordering::Relaxed) {
            call!(self.0; Command::Predict, p)
        } else {
            None
        }
    }

    #[inline]
    pub async fn drive(&self, p: Physical) {
        if self.1.load(Ordering::Relaxed) {
            let _ = self.0.send(Command::Drive(p)).await;
        }
    }
}

enum Command {
    Predict(Physical, Sender<Trajectory>),
    Drive(Physical),
}

pub(super) enum Event {
    Connected,
    Disconnected,

    StatusUpdated(PM1Status),
    OdometryUpdated(Instant, Odometry),
}

pub(super) fn supervisor() -> (Chassis, Receiver<Event>) {
    let (for_extern, command) = unbounded();
    let (event, to_extern) = unbounded();
    let is_available_clone = Arc::new(AtomicBool::new(false));
    let is_available = is_available_clone.clone();
    task::spawn_blocking(move || {
        let mut odometry = Odometry::ZERO;

        SupervisorForSingle::<PM1>::new().join(|e| {
            match e {
                Connected(_, chassis) => {
                    is_available.store(true, Ordering::Relaxed);
                    task::block_on(async {
                        join!(
                            send_async!(Event::Connected => event),
                            send_async!(Event::StatusUpdated(*chassis.status()) => event)
                        );
                    });
                }
                Disconnected => {
                    is_available.store(false, Ordering::Relaxed);
                    // 消费（拒绝）所有断连前到来，未及处理的消息
                    while let Ok(_) = command.try_recv() {}
                    task::block_on(send_async!(Event::Disconnected => event));
                }
                ConnectFailed => {
                    thread::sleep(Duration::from_secs(1));
                }
                Event(chassis, e) => {
                    if let Some((time, e)) = e {
                        if let PM1Event::Odometry(delta) = e {
                            odometry += delta;
                            task::block_on(
                                send_async!(Event::OdometryUpdated(time, odometry) => event),
                            );
                        } else {
                            task::block_on(
                                send_async!(Event::StatusUpdated(*chassis.status()) => event),
                            );
                        }
                    }

                    use Command::*;
                    let mut tgt = None;
                    let mut pre = None;
                    while let Ok(cmd) = command.try_recv() {
                        match cmd {
                            Drive(p) => tgt = Some(p),
                            Predict(_, _) => pre = Some(cmd),
                        }
                    }
                    if let Some(p) = tgt {
                        chassis.drive(p);
                    }
                    if let Some(Predict(p, r)) = pre {
                        let (model, predictor) = chassis.predict_with(p);
                        let tr: Trajectory = Box::new(TrajectoryPredictor { model, predictor });
                        task::block_on(send_async!(tr => r));
                    }
                }
            };
            true
        });
    });
    (Chassis(for_extern, is_available_clone), to_extern)
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
