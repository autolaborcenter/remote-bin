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
}
