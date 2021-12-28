#[cfg(feature = "runtime")]
mod atomic;

#[cfg(feature = "runtime")]
pub use atomic::AtomicDeviceCode;

/// 设备连接性的压缩编码
#[derive(Clone, Copy)]
pub struct DeviceCode(pub u32);

impl Default for DeviceCode {
    fn default() -> Self {
        Self(0)
    }
}
