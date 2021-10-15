use super::{chassis::Message as Chassis, Event};
use async_std::channel::{Receiver, Sender};
use parry2d::na::Isometry2;
use path_follower::Controller;
use pm1_sdk::model::Physical;
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use std::time::Instant;

pub(super) enum Message {
    Absolute(Instant, Isometry2<f32>),
    Relative(Instant, Isometry2<f32>),
    Record(String),
    Follow(String),
    Cancel,
    Pause(bool),
}

pub(super) async fn task(
    chassis: Sender<Chassis>,
    app: Sender<Event>,
    mail_box: Receiver<Message>,
) {
    let mut controller = Controller::new("path").unwrap();
    let mut filter = InterpolationAndPredictionFilter::new();
    let mut pause = false;

    while let Ok(msg) = mail_box.recv().await {
        use Message::*;
        match msg {
            Absolute(time, pose) => {
                let pose = filter.update(PoseType::Absolute, time, pose);
                let _ = app.send(Event::PoseUpdated(pose)).await;
                if let Some(proportion) = controller.put_pose(&pose) {
                    if !pause {
                        let _ = chassis.send(control(proportion)).await;
                    }
                }
            }
            Relative(time, pose) => {
                let pose = filter.update(PoseType::Relative, time, pose);
                let _ = app.send(Event::PoseUpdated(pose)).await;
                if let Some(proportion) = controller.put_pose(&pose) {
                    if !pause {
                        let _ = chassis.send(control(proportion)).await;
                    }
                }
            }
            Record(name) => {
                if let Err(e) = controller.record_to(name.as_str()) {
                    eprintln!("Failed to record: {:?}", e);
                }
            }
            Follow(name) => {
                if let Err(e) = controller.follow(name.as_str()) {
                    eprintln!("Failed to follow: {:?}", e);
                }
            }
            Cancel => {
                controller.stop_task();
            }
            Pause(value) => {
                pause = value;
            }
        };
    }
}

fn control(proportion: f32) -> Chassis {
    Chassis::PredictAutomatic(Physical {
        speed: 0.25,
        rudder: -2.0 * (proportion - 0.5),
    })
}
