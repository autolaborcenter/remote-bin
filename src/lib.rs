use parry2d::na::Isometry2;
use pm1_sdk::model::{ChassisModel, Physical, Predictor};
use std::{net::Ipv4Addr, time::Instant};

pub mod chassis;
pub mod lidar;
pub mod rtk;
pub mod tracker;

pub extern crate async_std;

pub enum Commander {
    Artificial,
    Automatic,
}

pub enum MsgToChassis {
    PrintStatus,
    Predict(Commander, Physical),
    Move(Physical),
}

pub enum MsgToLidar {
    Check(ChassisModel, Predictor),
    Send(Option<Ipv4Addr>),
}

pub enum MsgToFollower {
    Absolute(Instant, Isometry2<f32>),
    Relative(Instant, Isometry2<f32>),
    Record(String),
    Follow(String),
    Cancel,
    Pause(bool),
}
