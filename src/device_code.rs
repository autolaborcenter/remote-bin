/// 设备连接性
///
/// [rtk convergence][rtk initialized][rtk][急停按钮][底盘]
#[derive(Clone, Copy)]
pub struct DeviceCode(pub u32);

impl Default for DeviceCode {
    fn default() -> Self {
        Self(0)
    }
}

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

#[cfg(feature = "runtime")]
impl DeviceCode {
    #[inline]
    pub(super) fn set(&mut self, indices: &[u8]) -> Option<Self> {
        foreach!(indices; |i| self.set_bit(*i)).map(|_| *self)
    }

    #[inline]
    pub(super) fn clear(&mut self, indices: &[u8]) -> Option<Self> {
        foreach!(indices; |i| self.clear_bit(*i)).map(|_| *self)
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
