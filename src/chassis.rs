use super::{lidar::Message as Lidar, macros::send_anyway, tracker::Message as Tracker, Event};
use async_std::channel::{Receiver, Sender};
use pm1_sdk::{
    driver::{Driver, SupersivorEventForSingle::*, SupervisorForSingle},
    model::{Odometry, Physical},
    PM1Event, PM1,
};
use std::{
    thread,
    time::{Duration, Instant},
};

const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(200);

pub(super) enum Message {
    PredictArtificial(Physical),
    PredictAutomatic(Physical),
    Move(Physical),
}

pub(super) fn supervisor(
    lidar: Sender<Lidar>,
    tracker: Sender<Tracker>,
    app: Sender<Event>,
    mail_box: Receiver<Message>,
) {
    let mut artificial_deadline = Instant::now();
    let mut pose = Odometry::ZERO.pose;
    SupervisorForSingle::<PM1>::new().join(|e| {
        match e {
            Connected(_, driver) => eprintln!("Connected: {}", driver.status()),
            ConnectFailed => {
                eprintln!("Failed.");
                thread::sleep(Duration::from_secs(1));
            }
            Disconnected => {
                eprintln!("Disconnected.");
                thread::sleep(Duration::from_secs(1));
            }

            Event(pm1, e) => {
                if let Some((time, e)) = e {
                    if let PM1Event::Odometry(delta) = e {
                        pose *= delta.pose;
                        send_anyway!(Tracker::Relative(time, pose) => tracker);
                    } else {
                        send_anyway!(Event::ChassisStatusUpdated(*pm1.status()) => app);
                    }
                }
                while let Ok(msg) = mail_box.try_recv() {
                    use Message::*;
                    match msg {
                        PredictArtificial(p) => {
                            artificial_deadline = Instant::now() + ARTIFICIAL_TIMEOUT;
                            send_anyway!(predict(pm1, p) => lidar);
                        }
                        PredictAutomatic(p) => {
                            if Instant::now() > artificial_deadline {
                                send_anyway!(predict(pm1, p) => lidar);
                            }
                        }
                        Move(p) => pm1.send((Instant::now(), p)),
                    }
                }
            }
        };
        true
    });
}

fn predict(chassis: &PM1, target: Physical) -> Lidar {
    let (model, mut predictor) = chassis.predict();
    predictor.set_target(target);
    Lidar(model, predictor)
}
