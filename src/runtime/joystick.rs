use pm1_sdk::model::Physical;
use std::f32::consts::FRAC_PI_2;
use steering::{Device, Steering};

pub(super) struct Joystick(Device);

impl Joystick {
    pub fn new() -> Self {
        Self(Device::new())
    }

    pub fn get(&mut self) -> Physical {
        let steering::Status { level, rho, theta } = self.0.status();
        if rho < 0.005 {
            Physical::RELEASED
        } else {
            Physical {
                speed: rho * (level as f32) / 5.0,
                rudder: theta.signum() * (theta / FRAC_PI_2).powi(2) * FRAC_PI_2,
            }
        }
    }
}
