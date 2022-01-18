use super::DeviceCode;
use std::sync::{
    atomic::{AtomicU32, Ordering::*},
    Arc,
};

/// 连接性编码的原子版本
#[derive(Clone)]
pub struct AtomicDeviceCode(Arc<AtomicU32>);

macro_rules! foreach {
    ($indices:ident; $x:expr) => {
        $indices
            .into_iter()
            .map($x)
            .collect::<Vec<_>>()
            .into_iter()
            .find(|b| *b)
    };
}

impl DeviceCode {
    #[inline]
    fn set(&mut self, indices: &[u8]) -> Option<u32> {
        foreach!(indices; |i| self.set_bit(*i)).map(|_| self.0)
    }

    #[inline]
    fn clear(&mut self, indices: &[u8]) -> Option<u32> {
        foreach!(indices; |i| self.clear_bit(*i)).map(|_| self.0)
    }

    #[inline]
    fn set_bit(&mut self, i: u8) -> bool {
        let mask = 1 << i;
        if self.0 & mask == 0 {
            self.0 |= mask;
            true
        } else {
            false
        }
    }

    #[inline]
    fn clear_bit(&mut self, i: u8) -> bool {
        let mask = 1 << i;
        if self.0 & mask != 0 {
            self.0 &= !mask;
            true
        } else {
            false
        }
    }
}

impl Default for AtomicDeviceCode {
    fn default() -> Self {
        Self(Arc::new(AtomicU32::new(0)))
    }
}

impl AtomicDeviceCode {
    #[inline]
    pub(crate) fn set(&self, indices: &[u8]) -> Option<DeviceCode> {
        self.0
            .fetch_update(SeqCst, SeqCst, |last| DeviceCode(last).set(indices))
            .ok()
            .and_then(|c| DeviceCode(c).set(indices))
            .map(DeviceCode)
    }

    #[inline]
    pub(crate) fn clear(&self, indices: &[u8]) -> Option<DeviceCode> {
        self.0
            .fetch_update(SeqCst, SeqCst, |last| DeviceCode(last).clear(indices))
            .ok()
            .and_then(|c| DeviceCode(c).clear(indices))
            .map(DeviceCode)
    }
}

#[test]
fn test() {
    let mut code = DeviceCode::default();
    assert_eq!(Some(0b10), code.set(&[1]));
    assert_eq!(None, code.set(&[1]));
    assert_eq!(None, code.clear(&[0]));
    assert_eq!(Some(0), code.clear(&[1]));
}
