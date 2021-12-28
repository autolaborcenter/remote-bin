use async_std::{
    net::{ToSocketAddrs, UdpSocket},
    sync::Arc,
    task,
};
use gnss::Enu;
use monitor_tool::{palette, rgba, vertex, Encoder, Shape, Vertex};
use parry2d::na::Isometry2;
use pm1_sdk::model::Pm1Model;
use pose_filter::Particle;
use rtk_qxwz::GpggaStatus;
use std::time::Duration;

const FOCUS: &str = "focus";
const POSE: &str = "pose";
const GPS: &str = "gps";
const PARTICLES: &str = "particles";
const POSE_GROUP: [&str; 3] = [POSE, GPS, PARTICLES];

#[derive(Clone)]
pub(super) struct Painter(Arc<UdpSocket>);

impl Painter {
    /// 构造绘图接口
    #[inline]
    pub async fn new() -> Self {
        let socket = Arc::new(UdpSocket::bind("0.0.0.0:0").await.unwrap());
        send_config(socket.clone(), Duration::from_secs(5));
        Self(socket)
    }

    /// 绘图
    #[inline]
    pub async fn paint(&self, f: impl FnOnce(&mut Encoder) -> ()) {
        let _ = self.0.send(&Encoder::with(f)).await;
    }

    /// 绘制 gps 单点
    pub async fn paint_gps(&self, status: GpggaStatus, enu: Enu) {
        use GpggaStatus::*;
        let level = match status {
            无效解 | 用户输入 | 航位推算 | PPS | PPP => return,
            单点解 => 0,
            伪距差分 => 1,
            浮点解 => 2,
            固定解 => 3,
        };
        self.paint(|encoder| {
            encoder.topic(GPS).push(vertex!(level; enu.e,enu.n; 0));
        })
        .await;
    }

    /// 绘制粒子滤波器粒子和输出
    pub async fn paint_filter(&self, pose: Isometry2<f32>, particles: &[Particle<Pm1Model>]) {
        self.paint(|encoder| {
            let Isometry2 {
                translation: parry2d::na::Translation { vector },
                rotation,
            } = pose;
            encoder
                .topic(POSE)
                .push(vertex!(0; vector[0], vector[1] => rotation.angle(); 0));
            encoder.with_topic(PARTICLES, |mut topic| {
                topic.clear();
                topic.extend(particles.iter().map(|p| {
                    let Isometry2 {
                        translation: parry2d::na::Translation { vector },
                        rotation,
                    } = p.pose;
                    vertex!(0; vector[0], vector[1] => rotation.angle(); 0)
                }));
            });
        })
        .await;
    }

    /// 配置目标地址
    pub async fn connect<A: ToSocketAddrs>(&self, a: A) {
        if let Ok(()) = self.0.connect(a).await {
            let clear = Encoder::with(|encoder| {
                for topic in POSE_GROUP {
                    encoder.topic(topic).clear();
                }
            });
            let _ = self.0.send(clear.as_slice()).await;
        }
    }
}

fn send_config(socket: Arc<UdpSocket>, period: Duration) {
    let packet = Encoder::with(|figure| {
        figure.config_topic(FOCUS, 1, 1, &[(0, rgba!(BLACK; 0.0))], |_| {});
        // pose 组
        {
            figure.layer(POSE, &POSE_GROUP, None);
            figure.sync_set(POSE, &POSE_GROUP, Some(Duration::from_secs(120)));
            figure.config_topic(POSE, u32::MAX, 0, &[(0, rgba!(VIOLET; 0.05))], |_| {});
            figure.config_topic(PARTICLES, u32::MAX, 0, &[(0, rgba!(ORANGE; 0.2))], |_| {});
            figure.config_topic(
                GPS,
                u32::MAX,
                0,
                &[
                    (0, rgba!(RED; 0.25)),
                    (1, rgba!(YELLOW; 0.5)),
                    (2, rgba!(GREEN; 0.5)),
                ],
                |_| {},
            );
        }
    });
    task::spawn(async move {
        loop {
            let _ = socket.send(&packet).await;
            task::sleep(period).await;
        }
    });
}
