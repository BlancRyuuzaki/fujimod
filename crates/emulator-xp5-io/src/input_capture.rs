//! XP5 input-capture banks.
//! Per-channel active capture configuration refuses. The bank enable mask
//! retains writes; no samples, cause bits or deadlines are invented.
use fujimod_emulator_devices::{dma::{self, Direction}, Cx, Device};

pub struct TransmitSink;
impl dma::Port for TransmitSink {
    fn request(&self, direction: Direction, _burst: bool) -> Option<u64> {
        (direction == Direction::Transmit).then_some(0)
    }
    fn receive(&mut self, _now: u64, _bytes: u8) -> Result<u64, String> {
        Err("input-capture transmit sink cannot receive".into())
    }
    fn transmit(&mut self, _now: u64, _bytes: u8, _value: u64) -> Result<(), String> {
        // Immediate acceptance with no downstream effect is a session assumption.
        Ok(())
    }
}

pub struct InputCapture {
    base: u64,
    channels: u64,
    enable_mask: u32,
}
impl InputCapture {
    pub fn new(base: u64) -> Result<Self, String> {
        let channels = match base {
            0xff928000 => 5,
            0xff929000 => 3,
            _ => return Err("invalid input-capture bank".into()),
        };
        Ok(Self {
            base,
            channels,
            enable_mask: 0,
        })
    }
    fn allowed_mask(&self) -> u32 {
        if self.base == 0xff928000 { 0x1f } else { 0xe0 }
    }
    fn readable(&self, address: u64, size: u8) -> bool {
        let offset = address.wrapping_sub(self.base);
        size == 4 && offset / 0x20 < self.channels && matches!(offset % 0x20, 0 | 4 | 0xc | 0x10)
    }
}
impl Device for InputCapture {
    fn name(&self) -> &'static str {
        "input-capture"
    }
    fn read(&mut self, cx: &mut Cx<'_>, address: u64, size: u8) -> u64 {
        if size == 4 && address == self.base + 0xa0 {
            return u64::from(self.enable_mask);
        }
        if !self.readable(address, size) {
            cx.unmodeled();
        }
        0
    }
    fn write(&mut self, cx: &mut Cx<'_>, address: u64, size: u8, value: u64) {
        let offset = address.wrapping_sub(self.base);
        if size == 4 && offset == 0xa0 && value & !u64::from(self.allowed_mask()) == 0 {
            self.enable_mask = value as u32;
            return;
        }
        if size != 4 || offset / 0x20 >= self.channels {
            cx.unmodeled();
            return;
        }
        match offset % 0x20 {
            0 if value == 0 => {}
            0xc if value & !0x10001 == 0 => {}
            _ => cx.unmodeled(),
        }
    }
    fn pollable(&self, address: u64, size: u8) -> bool {
        size == 4 && address == self.base + 0xa0 || self.readable(address, size)
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({"base": self.base, "enable_mask": self.enable_mask})
    }
    fn restore(&mut self, value: &serde_json::Value) -> Result<(), String> {
        let base = value
            .get("base")
            .and_then(serde_json::Value::as_u64)
            .ok_or("input-capture checkpoint needs base")?;
        let mask = value
            .get("enable_mask")
            .and_then(serde_json::Value::as_u64)
            .ok_or("input-capture checkpoint needs enable mask")?;
        if base != self.base || mask & !u64::from(self.allowed_mask()) != 0 {
            return Err("invalid input-capture checkpoint".into());
        }
        self.enable_mask = mask as u32;
        Ok(())
    }
}
#[cfg(test)]
mod tests {
    use super::*;
    use fujimod_emulator_devices::test_host::recording::RecordingHost;
    #[test]
    fn bank_bounds_idle_readback_and_active_capture_refusal() {
        let mut host = RecordingHost::default();
        for (base, channels) in [(0xff928000, 5), (0xff929000, 3)] {
            let mut capture = InputCapture::new(base).unwrap();
            for channel in 0..channels {
                let mut cx = Cx::new(&mut host, 4, 0, 0);
                let at = base + channel * 0x20;
                capture.write(&mut cx, at, 4, 0);
                capture.write(&mut cx, at + 0xc, 4, 0x10001);
                for offset in [0, 4, 0xc, 0x10] {
                    assert_eq!(capture.read(&mut cx, at + offset, 4), 0);
                }
                assert_eq!(cx.refusal(), None);
            }
            let mut cx = Cx::new(&mut host, 0, 0, 0);
            let outside = if base == 0xff928000 { 0xc0 } else { channels * 0x20 };
            capture.read(&mut cx, base + outside, 4);
            assert!(cx.is_unmodeled());
            let mut cx = Cx::new(&mut host, 0, 0, 0);
            capture.write(&mut cx, base, 4, 1);
            assert!(cx.is_unmodeled());
            let mut cx = Cx::new(&mut host, 0, 0, 0);
            capture.read(&mut cx, base, 8);
            assert!(cx.is_unmodeled());
            assert_eq!(capture.next_deadline(0), None);
            let saved = capture.save();
            capture.restore(&saved).unwrap();
        }
        assert!(host.events.is_empty());
        assert!(host.irqs.is_empty());
        let mut six = InputCapture::new(0xff928000).unwrap();
        let mut cx = Cx::new(&mut host, 0, 0, 0);
        assert_eq!(six.read(&mut cx, 0xff9280a0, 4), 0);
        assert!(cx.refusal().is_none());
        six.write(&mut cx, 0xff9280a0, 4, 8);
        assert_eq!(six.read(&mut cx, 0xff9280a0, 4), 8);
        let saved = six.save();
        let mut restored = InputCapture::new(0xff928000).unwrap();
        restored.restore(&saved).unwrap();
        assert_eq!(restored.read(&mut cx, 0xff9280a0, 4), 8);
        six.read(&mut cx, 0xff9280c0, 4);
        assert!(cx.is_unmodeled());
        assert!(
            restored
                .restore(&serde_json::json!({"base":0xff929000u64,"enable_mask":8}))
                .is_err()
        );
    }
}
