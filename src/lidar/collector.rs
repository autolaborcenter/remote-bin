use async_std::{
    io::WriteExt,
    sync::{Arc, Mutex},
};
use lidar_faselase::{zip::PointZipped, Point};
use parry2d::math::{self, Real};
use std::f32::consts::PI;

pub(super) struct Collector {
    pub points: XXXXX,
    bits: Vec<Vec<u8>>,
    trans: (f32, f32, f32),
}

#[derive(Clone)]
pub(super) struct XXXXX(pub Arc<Mutex<Vec<Vec<math::Point<Real>>>>>);

impl Collector {
    pub fn new(trans: (f32, f32, f32)) -> Self {
        Self {
            points: XXXXX(Arc::new(Mutex::new(Vec::new()))),
            bits: Vec::new(),
            trans,
        }
    }

    pub async fn put(&mut self, i: usize, section: Vec<Point>) {
        let mut zipped = Vec::with_capacity(section.len() * std::mem::size_of::<PointZipped>());
        let mut transed = Vec::with_capacity(section.len());
        for Point { len, dir } in section {
            let _ = zipped.write_all(&PointZipped::new(len, dir).0).await;
            let (x, y, t) = self.trans;
            let len = len as f32 / 100.0;
            let dir = dir as f32 * 2.0 * PI / 5760.0 + t;
            let (sin, cos) = dir.sin_cos();
            transed.push(math::Point::new(cos * len + x, sin * len + y));
        }

        {
            let ref mut points = self.points.0.lock().await;
            if points.len() <= i {
                points.resize_with(i, || Vec::new());
            }
            points[i] = transed;
        }
        {
            let ref mut bits = self.bits;
            if bits.len() <= i {
                bits.resize_with(i + 1, || Vec::new());
            }
            bits[i] = zipped;
        }
    }

    pub async fn write_to(&self, buf: &mut Vec<u8>) {
        let len = self.bits.iter().map(|v| v.len()).sum::<usize>();
        buf.reserve(len + std::mem::size_of::<u16>());
        let _ = buf
            .write_all(unsafe {
                std::slice::from_raw_parts(
                    &(len as u16) as *const _ as *const u8,
                    std::mem::size_of::<u16>(),
                )
            })
            .await;
        for v in &self.bits {
            let _ = buf.write_all(&v);
        }
    }

    pub async fn clear(&mut self) {
        self.points.0.lock().await.clear();
        self.bits.clear();
    }
}
