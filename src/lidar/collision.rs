use super::{CollisionInfo, Duration, Odometry, Trajectory};
use parry2d::{
    math::{Point, Real},
    na::Isometry2,
    query::PointQuery,
    shape::ConvexPolygon,
};

pub(super) fn detect(
    frame: &[&Vec<Vec<Point<Real>>>],
    trajectory: Trajectory,
) -> Option<CollisionInfo> {
    let mut pose = Odometry::ZERO;
    let mut time = Duration::ZERO;
    let mut size = 1.0;
    for (dt, dp) in trajectory {
        time += dt;
        if time > Duration::from_secs(2) {
            return None;
        }
        size += dp.s;
        pose += dp;
        if check(&frame, pose.pose, size) {
            return Some(CollisionInfo(time, pose, 1.0 / size));
        }
    }
    panic!("Impossible!");
}

fn check(frame: &[&Vec<Vec<Point<Real>>>], pose: Isometry2<f32>, size: f32) -> bool {
    let outline = ROBOT_OUTLINE
        .iter()
        .map(|(x, y)| pose * Point::new(x * size, y * size))
        .collect();
    let outline = ConvexPolygon::from_convex_polyline(outline).unwrap();
    let aabb = outline.local_aabb();
    frame.iter().any(|c| {
        c.iter().any(|s| {
            s.iter()
                .filter(|p| aabb.contains_local_point(p))
                .any(|p| outline.contains_local_point(p))
        })
    })
}

const ROBOT_OUTLINE: [(f32, f32); 16] = [
    (0.25, 0.8),
    (0.12, 0.14),
    (0.10, 0.18),
    (0.10, 0.26),
    //
    (-0.10, 0.26),
    (-0.10, 0.18),
    (-0.25, 0.18),
    (-0.47, 0.12),
    //
    (-0.47, -0.12),
    (-0.25, -0.18),
    (-0.10, -0.18),
    (-0.10, -0.26),
    //
    (0.10, -0.26),
    (0.10, -0.18),
    (0.12, -0.14),
    (0.25, -0.8),
];
