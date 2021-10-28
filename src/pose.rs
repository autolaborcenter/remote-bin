use lidar_faselase::PointZipped;
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

impl Pose {
    pub const ZERO: Self = Self {
        x: 0.0,
        y: 0.0,
        theta: 0.0,
    };

    pub(crate) fn transform_u16(&self, len: u16, dir: u16) -> (f32, f32) {
        let Pose { x, y, theta } = self;
        let len = len as f32 / 100.0;
        let dir = dir as f32 * 2.0 * PI / 5760.0 + theta;
        let (sin, cos) = dir.sin_cos();
        (cos * len + x, sin * len + y)
    }

    #[inline]
    pub fn transform_zipped(&self, p: PointZipped) -> (f32, f32) {
        self.transform_u16(p.len(), p.dir())
    }
}
