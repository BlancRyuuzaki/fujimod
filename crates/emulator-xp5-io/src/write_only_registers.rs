//! Write-only register storage for explicitly mapped windows.
use fujimod_emulator_devices::{Cx, Device, checkpoint::Checkpoint};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

#[derive(Clone, Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
struct State {
    base: u64,
    size: u64,
    width: u8,
    registers: BTreeMap<u64, u32>,
}

pub struct WriteOnlyRegisters {
    state: State,
}

impl WriteOnlyRegisters {
    pub fn new(base: u64, size: u64, width: u8) -> Result<Self, String> {
        if !matches!(width, 2 | 4)
            || base % u64::from(width) != 0
            || size == 0
            || size % u64::from(width) != 0
            || base.checked_add(size).is_none()
        {
            return Err("invalid write-only register window".into());
        }
        Ok(Self {
            state: State {
                base,
                size,
                width,
                registers: BTreeMap::new(),
            },
        })
    }

    fn valid_register_offset(&self, offset: u64) -> bool {
        offset < self.state.size && offset % u64::from(self.state.width) == 0
    }

    fn max_value(&self) -> u64 {
        u64::from(u32::MAX >> (8 * (4 - self.state.width)))
    }

    fn validate_state(&self, state: &State) -> Result<(), String> {
        if state.base != self.state.base
            || state.size != self.state.size
            || state.width != self.state.width
            || state
                .registers
                .keys()
                .any(|offset| !self.valid_register_offset(*offset))
            || state
                .registers
                .values()
                .any(|value| u64::from(*value) > self.max_value())
        {
            return Err("invalid write-only register checkpoint".into());
        }
        Ok(())
    }
}

impl Device for WriteOnlyRegisters {
    fn name(&self) -> &'static str {
        "write-only-registers"
    }

    fn read(&mut self, cx: &mut Cx<'_>, _: u64, _: u8) -> u64 {
        cx.unmodeled();
        0
    }

    fn write(&mut self, cx: &mut Cx<'_>, address: u64, size: u8, value: u64) {
        let Some(offset) = address
            .checked_sub(self.state.base)
            .filter(|offset| self.valid_register_offset(*offset))
        else {
            cx.unmodeled();
            return;
        };
        if size != self.state.width || value > self.max_value() {
            cx.unmodeled();
            return;
        }
        self.state.registers.insert(offset, value as u32);
    }

    fn checkpoint(&self) -> Result<Checkpoint, String> {
        Checkpoint::encode(self.name(), &self.state)
    }

    fn validate_checkpoint(&self, checkpoint: &Checkpoint) -> Result<(), String> {
        self.validate_state(&checkpoint.decode(self.name())?)
    }

    fn restore_checkpoint(&mut self, checkpoint: &Checkpoint) -> Result<(), String> {
        let state: State = checkpoint.decode(self.name())?;
        self.validate_state(&state)?;
        self.state = state;
        Ok(())
    }

    fn fingerprint(&self) -> Result<Value, String> {
        Ok(serde_json::to_value(&self.state).unwrap())
    }

    fn save(&self) -> Value {
        serde_json::to_value(&self.state).unwrap()
    }

    fn restore(&mut self, value: &Value) -> Result<(), String> {
        let state: State =
            serde_json::from_value(value.clone()).map_err(|error| error.to_string())?;
        self.validate_state(&state)?;
        self.state = state;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use fujimod_emulator_devices::test_host::recording::RecordingHost;

    #[test]
    fn selected_halfword_width_preserves_write_only_state_and_word_default() {
        let mut host = RecordingHost::default();
        let mut halfwords = WriteOnlyRegisters::new(0x1000, 0x20, 2).unwrap();
        halfwords.write(&mut Cx::new(&mut host, 0, 0, 0), 0x1002, 2, 0xabcd);
        let before_refusal = halfwords.save();

        let mut cx = Cx::new(&mut host, 0, 0, 0);
        halfwords.write(&mut cx, 0x1001, 2, 1);
        assert!(cx.is_unmodeled());
        assert_eq!(halfwords.save(), before_refusal);

        let mut cx = Cx::new(&mut host, 0, 0, 0);
        halfwords.write(&mut cx, 0x1004, 4, 1);
        assert!(cx.is_unmodeled());
        let mut cx = Cx::new(&mut host, 0, 0, 0);
        halfwords.write(&mut cx, 0x1004, 2, 0x1_0000);
        assert!(cx.is_unmodeled());

        let mut cx = Cx::new(&mut host, 0, 0, 0);
        assert_eq!(halfwords.read(&mut cx, 0x1002, 2), 0);
        assert!(cx.is_unmodeled());

        let checkpoint = halfwords.checkpoint().unwrap();
        let mut restored = WriteOnlyRegisters::new(0x1000, 0x20, 2).unwrap();
        restored.validate_checkpoint(&checkpoint).unwrap();
        restored.restore_checkpoint(&checkpoint).unwrap();
        assert_eq!(restored.save(), before_refusal);
        let mut invalid = before_refusal.clone();
        invalid["registers"]["2"] = Value::from(0x1_0000);
        assert!(restored.restore(&invalid).is_err());
        assert_eq!(restored.save(), before_refusal);

        let mut words = WriteOnlyRegisters::new(0x2000, 0x20, 4).unwrap();
        words.write(&mut Cx::new(&mut host, 0, 0, 0), 0x2004, 4, 0x1234_5678);
        assert_eq!(words.save()["registers"]["4"], 0x1234_5678);
        assert!(WriteOnlyRegisters::new(0x1001, 0x20, 2).is_err());
        assert!(WriteOnlyRegisters::new(0x1000, 0x20, 1).is_err());
    }
}
