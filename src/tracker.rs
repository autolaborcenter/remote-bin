use crate::Commander;

use super::{
    MsgToChassis,
    MsgToFollower::{self, *},
};
use async_std::channel::{Receiver, Sender};
use path_follower::Controller;
use pm1_sdk::model::Physical;
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};

pub async fn task(to_chassis: Sender<MsgToChassis>, mail_box: Receiver<MsgToFollower>) {
    let mut controller = Controller::new("path").unwrap();
    let mut filter = InterpolationAndPredictionFilter::new();
    let mut pause = false;
    while let Ok(msg) = mail_box.recv().await {
        match msg {
            Absolute(time, pose) => {
                if let Some(proportion) =
                    controller.put_pose(&filter.update(PoseType::Absolute, time, pose))
                {
                    if !pause {
                        let _ = to_chassis.send(control(proportion)).await;
                    }
                }
            }
            Relative(time, pose) => {
                if let Some(proportion) =
                    controller.put_pose(&filter.update(PoseType::Relative, time, pose))
                {
                    if !pause {
                        let _ = to_chassis.send(control(proportion)).await;
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

fn control(proportion: f32) -> MsgToChassis {
    MsgToChassis::Predict(
        Commander::Automatic,
        Physical {
            speed: 0.25,
            rudder: -2.0 * (proportion - 0.5),
        },
    )
}
