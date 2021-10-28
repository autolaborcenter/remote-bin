#[derive(Clone, Copy)]
pub struct DeviceCode(pub u32);

impl Default for DeviceCode {
    fn default() -> Self {
        Self(0)
    }
}

impl DeviceCode {
    pub(super) fn set(&mut self, i: usize) -> Option<Self> {
        let mask = 1 << i;
        if self.0 & mask == 0 {
            self.0 |= mask;
            Some(*self)
        } else {
            None
        }
    }

    pub(super) fn clear(&mut self, i: usize) -> Option<Self> {
        let mask = 1 << i;
        if self.0 & mask != 0 {
            self.0 &= !mask;
            Some(*self)
        } else {
            None
        }
    }
}
