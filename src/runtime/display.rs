use async_std::{
    net::{ToSocketAddrs, UdpSocket},
    sync::Arc,
    task,
};
use monitor_tool::{palette, rgba, Encoder};
use std::time::Duration;

pub(super) const FOCUS: &str = "focus";
pub(super) const POSE: &str = "pose";
pub(super) const GPS: &str = "gps";
pub(super) const PARTICLES: &str = "particles";
pub(super) const POSE_GROUP: [&str; 3] = [POSE, GPS, PARTICLES];

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

    /// 配置目标地址
    pub async fn connect<A: ToSocketAddrs>(&self, a: A) {
        let _ = self.0.connect(a).await;
        let clear = Encoder::with(|encoder| {
            for topic in POSE_GROUP {
                encoder.topic(topic).clear();
            }
        });
        let _ = self.0.send(clear.as_slice()).await;
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
