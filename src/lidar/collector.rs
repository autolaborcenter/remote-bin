﻿use super::Pose;
use crate::{CollisionInfo, Trajectory};
use async_std::sync::{Arc, Mutex};
use lidar_faselase::{zip::PointZipped, Point};
use parry2d::{
    math::{self, Real},
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
            points.resize_with(i, || Vec::new());
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
        let mut odom = Odometry::ZERO;
        let mut time = Duration::ZERO;
        let mut size = 1.0;
        for (dt, dp) in trajectory {
            time += dt;
            if time > Duration::from_secs(2) {
                return None;
            }
            size += dp.s;
            odom += dp;
            // 检测碰撞
            let any = {
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
                frame.iter().any(|c| {
                    c.iter().any(|s| {
                        s.iter()
                            .filter(|p| aabb.contains_local_point(p))
                            .any(|p| outline.contains_local_point(p))
                    })
                })
            };

            if any {
                return Some(CollisionInfo(time, odom, 1.0 / size));
            }
        }
        panic!("Impossible!");
    }
}

fn extend_data<T: Sized>(s: &mut Vec<u8>, t: T) {
    s.extend(unsafe {
        std::slice::from_raw_parts(&t as *const _ as *const u8, std::mem::size_of::<T>())
    });
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
