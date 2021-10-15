use lidar_faselase::FrameCollector;
use parry2d::{
    math::{Point, Real},
    na::Isometry2,
    query::PointQuery,
    shape::ConvexPolygon,
};
use pm1_sdk::model::{ChassisModel, Odometry, Physical, Predictor};
use std::time::Duration;

pub(super) fn detect(
    frame: &[FrameCollector],
    model: ChassisModel,
    predictor: Predictor,
) -> Option<(Duration, Odometry, f32)> {
    const PERIOD: Duration = Duration::from_millis(40);
    const PERIOD_SEC: f32 = 0.04;

    let mut pose = Odometry::ZERO;
    let mut time = Duration::ZERO;
    let mut size = 1.0;
    for status in predictor {
        time += PERIOD;
        if time > Duration::from_secs(2) {
            return None;
        }
        let delta = model.physical_to_odometry(Physical {
            speed: status.speed * PERIOD_SEC,
            ..status
        });
        size += delta.s;
        pose += delta;
        if check(&frame, pose.pose, size) {
            return Some((time, pose, size));
        }
    }
    panic!("Impossible!");
}

fn check(frame: &[FrameCollector], pose: Isometry2<f32>, size: f32) -> bool {
    let outline = ROBOT_OUTLINE
        .iter()
        .map(|p| pose * transform(*p, size))
        .collect();
    let outline = ConvexPolygon::from_convex_polyline(outline).unwrap();
    let aabb = outline.local_aabb();
    frame.iter().any(|c| {
        c.sections.iter().any(|s| {
            s.1.iter()
                .map(|p| transform(*p, 1.0))
                .filter(|p| aabb.contains_local_point(p))
                .any(|p| outline.contains_local_point(&p))
        })
    })
}

const ROBOT_OUTLINE: [(i16, i16); 16] = [
    (250, 80),
    (120, 140),
    (100, 180),
    (100, 260),
    //
    (-100, 260),
    (-100, 180),
    (-250, 180),
    (-470, 120),
    //
    (-470, -120),
    (-250, -180),
    (-100, -180),
    (-100, -260),
    //
    (100, -260),
    (100, -180),
    (120, -140),
    (250, -80),
];

fn transform(p: (i16, i16), mut size: f32) -> Point<Real> {
    size /= 1000.0;
    Point::new(p.0 as f32 * size, p.1 as f32 * size)
}
