use super::super::{CollisionInfo, Trajectory};
use crate::Pose;
use async_std::sync::{Arc, Mutex};
use lidar_faselase::{Point, PointZipped};
use parry2d::{
    math::{self, Real},
    na::Vector2,
    query::PointQuery,
    shape::ConvexPolygon,
};
use pm1_sdk::model::Odometry;
use std::time::Duration;

#[derive(Clone)]
pub(super) struct Group(Vec<Points>);

pub(super) struct Collector {
    points: Points,
    bits: Vec<Vec<u8>>,
    trans: Pose,
}

#[cfg(debug_assertions)]
mod c {
    pub const DETECT_STEP_S: f32 = 0.1;
    pub const DETECT_STEP_A: f32 = std::f32::consts::PI / 9.0;
}

#[cfg(not(debug_assertions))]
mod c {
    pub const DETECT_STEP_S: f32 = 0.05;
    pub const DETECT_STEP_A: f32 = std::f32::consts::PI / 18.0;
}

type Points = Arc<Mutex<Vec<Vec<math::Point<Real>>>>>;

impl Collector {
    pub async fn put(&mut self, i: usize, section: Vec<Point>) {
        // 变换
        let mut zipped = Vec::with_capacity(section.len() * std::mem::size_of::<PointZipped>());
        let mut transed = Vec::with_capacity(section.len());
        for Point { len, dir } in section {
            let _ = zipped.extend(&PointZipped::new(len, dir).0);
            let (x, y) = self.trans.transform_u16(len, dir);
            transed.push(math::Point::new(x, y));
        }
        // 保存编码
        let ref mut bits = self.bits;
        if bits.len() <= i {
            bits.resize_with(i + 1, || Vec::new());
        }
        bits[i] = zipped;
        // 保存点云
        let ref mut points = self.points.lock().await;
        if points.len() <= i {
            points.resize_with(i + 1, || Vec::new());
        }
        points[i] = transed;
    }

    /// 将全部编码写入到缓冲区
    pub fn write_to(&self, buf: &mut Vec<u8>) {
        let len = self.bits.iter().map(|v| v.len()).sum::<usize>();
        buf.reserve(len + std::mem::size_of::<u16>());
        let _ = extend_data(buf, len as u16);
        let _ = extend_data(buf, self.trans);
        self.bits.iter().for_each(|v| buf.extend(v));
    }

    /// 清空
    pub async fn clear(&mut self) {
        self.points.lock().await.clear();
        self.bits.clear();
    }
}

impl Group {
    pub fn build(trans: &[Pose]) -> (Self, Vec<Collector>) {
        let collectors = trans
            .iter()
            .map(|trans| Collector {
                points: Arc::new(Mutex::new(Vec::new())),
                bits: Vec::new(),
                trans: *trans,
            })
            .collect::<Vec<_>>();
        (
            Self(collectors.iter().map(|c| c.points.clone()).collect()),
            collectors,
        )
    }

    pub async fn detect(&self, trajectory: Trajectory) -> Option<CollisionInfo> {
        // 锁定整个点云
        let mut frame = Vec::with_capacity(self.0.len());
        for x in &self.0 {
            frame.push(x.lock().await)
        }
        // 迭代路径
        let mut time = Duration::ZERO;
        let mut odom = Odometry::ZERO;
        let mut sub_odom = Odometry::ZERO;
        for (dt, dp) in trajectory {
            time += dt;
            if time > Duration::from_secs(2) {
                break;
            }
            sub_odom += dp;
            if sub_odom.s < c::DETECT_STEP_S && sub_odom.a < c::DETECT_STEP_A {
                continue;
            }
            odom += std::mem::replace(&mut sub_odom, Odometry::ZERO);
            let size = odom.s + 1.0;
            // 根据运行距离扩大轮廓
            let outline = ROBOT_OUTLINE
                .iter()
                .map(|(x, y)| odom.pose * math::Point::new(x * size, y * size))
                .collect();
            // 生成多边形
            let outline = ConvexPolygon::from_convex_polyline(outline).unwrap();
            // 计算包装盒，降低检测复杂度
            let aabb = outline.local_aabb();
            // 遍历检测碰撞
            if frame
                .iter()
                .flat_map(|c| c.iter().flatten())
                .any(|p| aabb.contains_local_point(p) && outline.contains_local_point(p))
            {
                // 检测到碰撞
                let risk = 1.0 / size;

                // 计算斥力
                let inv = odom.pose.inverse();
                let force =
                // 如果有一定规避空间
                if risk < 1.0 {
                    let (l, r) =
                    // 展平
                    frame.iter().flat_map(|c| c.iter().flatten())
                    // 累加障碍点数和合力
                    .fold(
                        (Force::ZERO, Force::ZERO),
                        |(mut l, mut r), p| {
                            let v = (inv * p).coords;
                            let squared = v.norm_squared();
                            if squared < 1.0 {
                                if v[1] > 0.0 {
                                    l.plus(-v / squared);
                                } else {
                                    r.plus(-v / squared);
                                }
                            }
                            (l, r)
                        },
                    );
                    // 归一化
                    (l.get() + r.get()) / size
                }
                // 没有规避空间，不必计算
                else {
                    Vector2::new(0.0, 0.0)
                };

                // 生成碰撞信息
                return Some(CollisionInfo {
                    time,
                    pose: odom,
                    risk,
                    force,
                });
            }
        }
        None
    }
}

struct Force {
    count: usize,
    value: Vector2<f32>,
}

impl Force {
    const ZERO: Self = Self {
        count: 0,
        value: Vector2::new(0.0, 0.0),
    };

    fn plus(&mut self, v: Vector2<f32>) {
        self.count += 1;
        self.value += v;
    }

    fn get(self) -> Vector2<f32> {
        self.value / self.count as f32
    }
}

#[inline]
fn extend_data<T: Sized>(s: &mut Vec<u8>, t: T) {
    s.extend(unsafe {
        std::slice::from_raw_parts(&t as *const _ as *const u8, std::mem::size_of::<T>())
    });
}

const ROBOT_OUTLINE: [(f32, f32); 16] = [
    (0.25, 0.08),
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
    (0.25, -0.08),
];
