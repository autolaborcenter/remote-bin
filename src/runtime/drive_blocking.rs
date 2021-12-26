use async_std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

const JOYSTICK_TIMEOUT: Duration = Duration::from_millis(500); // 手柄控制保护期
const ARTIFICIAL_TIMEOUT: Duration = Duration::from_millis(500); // 人工控制保护期

/// 处理各种控制方式的优先级
#[derive(Clone)]
pub(super) struct DriveBlocking {
    artificial_deadline: Arc<Mutex<Instant>>,
    joystick_deadline: Arc<Mutex<Instant>>,
}

impl DriveBlocking {
    #[inline]
    pub fn new() -> Self {
        let now = Instant::now();
        Self {
            artificial_deadline: Arc::new(Mutex::new(now)),
            joystick_deadline: Arc::new(Mutex::new(now)),
        }
    }

    #[inline]
    pub async fn drive_joystick(&self) {
        let now = Instant::now();
        *self.joystick_deadline.lock().await = now + JOYSTICK_TIMEOUT;
    }

    #[inline]
    pub async fn try_drive_automatic(&self) -> bool {
        let now = Instant::now();
        now > std::cmp::max(
            *self.joystick_deadline.lock().await,
            *self.artificial_deadline.lock().await,
        )
    }

    #[inline]
    pub async fn try_drive_artificial(&self) -> bool {
        let now = Instant::now();
        if now < *self.joystick_deadline.lock().await {
            return false;
        }
        *self.artificial_deadline.lock().await = now + ARTIFICIAL_TIMEOUT;
        true
    }
}
