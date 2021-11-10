use crate::{Point, CONFIG};
use parry2d::na::Isometry2;
use std::f32::consts::PI;

#[derive(Clone, Copy, PartialEq, Debug)]
pub struct Pose {
    pub x: f32,
    pub y: f32,
    pub theta: f32,
}

impl From<Isometry2<f32>> for Pose {
    fn from(src: Isometry2<f32>) -> Self {
        Self {
            x: src.translation.vector[0],
            y: src.translation.vector[1],
            theta: src.rotation.angle(),
        }
    }
}

const METER_LEN: f32 = 1.0 / CONFIG.len_meter as f32;
const RAD_DIR: f32 = 2.0 * PI / CONFIG.dir_round as f32;

impl Pose {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };

    pub fn transform_point(&self, p: Point) -> (f32, f32) {
        let Pose { x, y, theta } = self;
        let len = p.len as f32 * METER_LEN;
        let dir = p.dir as f32 * RAD_DIR + theta;
        let (sin, cos) = dir.sin_cos();
        (cos * len + x, sin * len + y)
    }
}
