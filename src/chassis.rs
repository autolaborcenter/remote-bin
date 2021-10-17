use super::{macros::send_anyway, Trajectory};
use async_std::{
    channel::{unbounded, Receiver, Sender},
    task,
};
use pm1_sdk::{
    driver::{Driver, SupersivorEventForSingle::*, SupervisorForSingle},
    model::{ChassisModel, Odometry, Physical, Predictor},
    PM1Event, PM1Status, CONTROL_PERIOD, PM1,
};
use std::{
    thread,
    time::{Duration, Instant},
};

const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(200);

pub(super) enum Command {
    PredictArtificial(Physical, Sender<Trajectory>),
    PredictAutomatic(Physical, Sender<Trajectory>),
    Move(Physical),
}

pub(super) enum Event {
    Connected,
    Disconnected,

    StatusUpdated(PM1Status),
    OdometryUpdated(Instant, Odometry),
}

pub(super) fn supervisor() -> (Sender<Command>, Receiver<Event>) {
    let (for_extern, command) = unbounded();
    let (event, to_extern) = unbounded();
    task::spawn_blocking(move || {
        let mut artificial_deadline = Instant::now();
        let mut odometry = Odometry::ZERO;

        SupervisorForSingle::<PM1>::new().join(|e| {
            match e {
                Connected(_, chassis) => {
                    send_anyway!(Event::Connected => event);
                    send_anyway!(Event::StatusUpdated(*chassis.status()) => event);
                }
                Disconnected => {
                    send_anyway!(Event::Disconnected => event);
                }
                ConnectFailed => {
                    thread::sleep(Duration::from_secs(1));
                }
                Event(chassis, e) => {
                    if let Some((time, e)) = e {
                        if let PM1Event::Odometry(delta) = e {
                            odometry += delta;
                            send_anyway!(Event::OdometryUpdated(time, odometry) => event);
                        } else {
                            send_anyway!(Event::StatusUpdated(*chassis.status()) => event);
                        }
                    }

                    use Command::*;

                    let (mov, pre) = receive_from(&command);
                    let now = Instant::now();
                    if let Some(p) = mov {
                        chassis.send((now, p));
                    }
                    match pre {
                        Some(PredictArtificial(p, r)) => {
                            artificial_deadline = now + ARTIFICIAL_TIMEOUT;
                            predict(chassis, p, r);
                        }
                        Some(PredictAutomatic(p, r)) => {
                            if now > artificial_deadline {
                                predict(chassis, p, r);
                            }
                        }
                        _ => {}
                    }
                }
            };
            true
        });
    });
    (for_extern, to_extern)
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

fn predict(chassis: &PM1, target: Physical, mail_box: Sender<Trajectory>) {
    let (model, mut predictor) = chassis.predict();
    predictor.set_target(target);
    send_anyway!(Box::new(TrajectoryPredictor{model,predictor}) => mail_box);
}

fn receive_from(command: &Receiver<Command>) -> (Option<Physical>, Option<Command>) {
    use Command::*;
    let mut mov = None;
    let mut pre = None;
    while let Ok(cmd) = command.try_recv() {
        match cmd {
            Move(p) => mov = Some(p),
            PredictArtificial(_, _) => pre = Some(cmd),
            PredictAutomatic(_, _) => {
                if let Some(PredictAutomatic(_, _)) | None = pre {
                    pre = Some(cmd);
                }
            }
        }
    }
    (mov, pre)
}
