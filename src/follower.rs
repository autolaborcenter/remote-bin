use super::{
    MsgToChassis,
    MsgToFollower::{self, *},
};
use pose_filter::{InterpolationAndPredictionFilter, PoseFilter, PoseType};
use std::sync::mpsc::{Receiver, Sender};

pub(super) fn task(to_chassis: Sender<MsgToChassis>, mail_box: Receiver<MsgToFollower>) {
    let mut filter = InterpolationAndPredictionFilter::new();

    for msg in mail_box {
        match msg {
            Absolute(time, pose) => filter.update(PoseType::Absolute, time, pose),
            Relative(time, pose) => filter.update(PoseType::Relative, time, pose),
        };
    }

     // for msg in receiver {
    //     match msg {
    //         Message::Pose(pose) => {
    //             if let Some(proportion) = controller.put_pose(&pose) {
    //                 *target.lock().unwrap() = (
    //                     Instant::now(),
    //                     Physical {
    //                         speed: 0.25,
    //                         rudder: -2.0 * (proportion - 0.5),
    //                     },
    //                 );
    //             }
    //         }
    //         Message::List => {
    //             for name in controller.list() {
    //                 println!("{}", name);
    //             }
    //         }
    //         Message::Record(name) => match controller.record_to(name.as_str()) {
    //             Ok(_) => println!("Done"),
    //             Err(e) => eprintln!("Failed: {}", e),
    //         },
    //         Message::Follow(name) => match controller.follow(name.as_str()) {
    //             Ok(_) => println!("Done"),
    //             Err(e) => eprintln!("Failed: {}", e),
    //         },
    //         Message::Stop => {
    //             controller.stop_task();
    //             println!("Done");
    //         }
    //         Message::Exit => break,
    //     }
    // }
}
