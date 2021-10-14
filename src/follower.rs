use crate::Commander;

use super::{
    MsgToChassis,
    MsgToFollower::{self, *},
};
use path_follower::Controller;
use pm1_sdk::model::Physical;
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use std::sync::mpsc::{Receiver, Sender};

pub(super) fn task(to_chassis: Sender<MsgToChassis>, mail_box: Receiver<MsgToFollower>) {
    let mut controller = Controller::new("path").unwrap();
    let mut filter = InterpolationAndPredictionFilter::new();
    let mut pause = false;

    for msg in mail_box {
        let mut control = |&pose| {
            if let Some(proportion) = controller.put_pose(&pose) {
                if !pause {
                    let _ = to_chassis.send(control(proportion));
                }
            }
        };

        match msg {
            Absolute(time, pose) => {
                control(&filter.update(PoseType::Absolute, time, pose));
            }
            Relative(time, pose) => {
                control(&filter.update(PoseType::Relative, time, pose));
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

fn control(proportion: f32) -> MsgToChassis {
    MsgToChassis::Predict(
        Commander::Automatic,
        Physical {
            speed: 0.25,
            rudder: -2.0 * (proportion - 0.5),
        },
    )
}
