mod device_code;
mod pose;

#[cfg(feature = "runtime")]
mod runtime;

#[cfg(feature = "runtime")]
pub use runtime::*;

pub const LOCAL_ORIGIN: WGS84 = WGS84 {
    latitude: 0.0,
    longitude: 0.0,
    altitude: 0.0,
};

pub use device_code::DeviceCode;
pub use gnss::{Enu, LocalReference, WGS84};
pub use lidar_ld19::{unzip, Point, CONFIG};
pub use pm1_sdk::model::{Odometry, Physical};
pub use pose::Pose;
