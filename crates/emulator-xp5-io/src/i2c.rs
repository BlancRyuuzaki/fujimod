//! I2C1 battery/power clients. Address36 is the gauge;66/68 retain
//! power-control writes.
//! Other addresses NACK. No device identity or unsolicited ID is invented.
use fujimod_emulator_devices::{Cx, Device};
use fujimod_emulator_parts_io::power_controller::FuelGauge;
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeMap;

#[derive(Serialize, Deserialize)]
pub struct BatteryI2c {
    regs: [u32; 8],
    gauge: FuelGauge,
    power: BTreeMap<u16, u8>,
    address: u8,
    register: u8,
    active: bool,
    reading: bool,
    indexed: bool,
    generation: u64,
    pending: Option<(bool, u8, bool)>,
    transfers: u64,
}
impl Default for BatteryI2c {
    fn default() -> Self {
        Self {
            regs: [0; 8],
            gauge: FuelGauge::new(),
            power: BTreeMap::new(),
            address: 0,
            register: 0,
            active: false,
            reading: false,
            indexed: false,
            generation: 0,
            pending: None,
            transfers: 0,
        }
    }
}
impl BatteryI2c {
    fn known_address(&self) -> bool {
        matches!(self.address, 0x36 | 0x66 | 0x68)
    }
    fn byte(&mut self, cx: &mut Cx<'_>, byte: u8) {
        if !self.indexed {
            self.register = byte;
            self.indexed = true;
            return;
        }
        let result = if self.address == FuelGauge::ADDRESS {
            self.gauge.write(self.register, byte)
        } else {
            self.power
                .insert((self.address as u16) << 8 | self.register as u16, byte);
            Ok(())
        };
        if let Err(error) = result {
            cx.fail(error);
        }
        self.register = self.register.wrapping_add(1);
    }
    fn receive(&mut self, cx: &mut Cx<'_>) -> u8 {
        let result = if self.address == FuelGauge::ADDRESS {
            self.gauge.read(self.register)
        } else {
            self.power
                .get(&((self.address as u16) << 8 | self.register as u16))
                .copied()
                .ok_or_else(|| {
                    format!(
                        "unmodeled power I2C {:02x}:{:02x}",
                        self.address, self.register
                    )
                })
        };
        self.register = self.register.wrapping_add(1);
        match result {
            Ok(value) => value,
            Err(error) => {
                cx.fail(error);
                0
            }
        }
    }
}
impl Device for BatteryI2c {
    fn name(&self) -> &'static str {
        "i2c1-battery-power"
    }
    fn read(&mut self, cx: &mut Cx<'_>, address: u64, size: u8) -> u64 {
        let offset = address.wrapping_sub(0xff925000);
        if !matches!(size, 1 | 4) || offset > 0x1c || offset & 3 != 0 {
            cx.unmodeled();
            return 0;
        }
        //1472880 recovery checks SDA/SCL at bits4/5, idle high.
        if offset == 0x1c {
            return (self.regs[7] | 0x30) as u64;
        }
        self.regs[offset as usize / 4] as u64
    }
    fn write(&mut self, cx: &mut Cx<'_>, address: u64, size: u8, value: u64) {
        let offset = address.wrapping_sub(0xff925000);
        if !matches!(size, 1 | 4) || offset > 0x1c || offset & 3 != 0 {
            cx.unmodeled();
            return;
        }
        self.regs[offset as usize / 4] = value as u32;
        if offset != 4 {
            return;
        }
        cx.set_irq(289, false);
        self.generation += 1;
        self.pending = None;
        let control = value as u32;
        if control & 0x12 != 0x12 {
            self.active = false;
            self.regs[0] = 0;
            return;
        }
        let address = !self.active || control & 0x20 != 0;
        self.pending = Some((address, self.regs[4] as u8, control & 8 != 0));
        // Functional byte budget, not a measured bus rate. At 20us, the
        // A 1ms interval completes the declared setup-key workload.
        cx.schedule(cx.now() + 1_000_000, self.generation);
    }
    fn event(&mut self, cx: &mut Cx<'_>, token: u64) {
        if token != self.generation {
            return;
        }
        let Some((address, byte, ack)) = self.pending.take() else {
            return;
        };
        if address {
            self.address = byte >> 1;
            self.reading = byte & 1 != 0;
            self.active = true;
            if !self.reading {
                self.indexed = false;
            }
            //Read-address ACK is not a data byte (ISR tests status bit0).
            self.regs[0] = if self.reading { 1 } else { 8 };
        } else if self.reading {
            self.regs[4] = self.receive(cx) as u32;
            self.regs[0] = if ack { 0 } else { 0x10 };
        } else {
            if self.known_address() {
                self.byte(cx, byte);
            }
            self.regs[0] = 8;
        }
        if !self.known_address() {
            self.regs[0] |= 0x10;
        }
        self.transfers += 1;
        self.regs[1] |= 0x10;
        if self.regs[1] & 0x40 != 0 {
            cx.set_irq(289, true);
        }
    }
    fn save(&self) -> Value {
        serde_json::to_value(self).unwrap()
    }
    fn restore(&mut self, state: &Value) -> Result<(), String> {
        *self = serde_json::from_value(state.clone())
            .map_err(|e| format!("invalid I2C battery checkpoint: {e}"))?;
        Ok(())
    }
}

/// Register-only controller for an explicitly declared idle bus.
#[derive(Serialize, Deserialize)]
pub struct IdleI2c {
    base: u64,
    idle_lines: u32,
    regs: [u32; 8],
}

impl IdleI2c {
    pub fn new(base: u64, idle_lines: u32) -> Result<Self, String> {
        if base & 3 != 0 || idle_lines & !0x30 != 0 {
            return Err("invalid idle I2C configuration".into());
        }
        Ok(Self {
            base,
            idle_lines,
            regs: [0; 8],
        })
    }

    fn index(&self, address: u64, size: u8) -> Option<usize> {
        let offset = address.checked_sub(self.base)?;
        (matches!(size, 1 | 4) && offset <= 0x1c && offset & 3 == 0).then_some(offset as usize / 4)
    }
}

impl Device for IdleI2c {
    fn name(&self) -> &'static str {
        "i2c-idle"
    }

    fn read(&mut self, cx: &mut Cx<'_>, address: u64, size: u8) -> u64 {
        let Some(index) = self.index(address, size) else {
            cx.unmodeled();
            return 0;
        };
        let value = if index == 7 {
            (self.regs[index] & !0x30) | self.idle_lines
        } else {
            self.regs[index]
        };
        if size == 1 {
            u64::from(value & 0xff)
        } else {
            u64::from(value)
        }
    }

    fn write(&mut self, cx: &mut Cx<'_>, address: u64, size: u8, value: u64) {
        let Some(index) = self.index(address, size) else {
            cx.unmodeled();
            return;
        };
        let value = if size == 1 {
            (self.regs[index] & !0xff) | (value as u32 & 0xff)
        } else {
            value as u32
        };
        if index == 1 && value & 0x12 == 0x12 {
            cx.unmodeled();
            return;
        }
        self.regs[index] = value;
    }

    fn save(&self) -> Value {
        serde_json::to_value(self).unwrap()
    }
    fn restore(&mut self, state: &Value) -> Result<(), String> {
        let saved: Self = serde_json::from_value(state.clone())
            .map_err(|e| format!("invalid idle I2C checkpoint: {e}"))?;
        if saved.base != self.base || saved.idle_lines != self.idle_lines {
            return Err("idle I2C checkpoint configuration differs".into());
        }
        self.regs = saved.regs;
        Ok(())
    }
}

/// Byte-oriented controller with explicitly selected register clients.
#[derive(Serialize, Deserialize)]
pub struct RegisterClientsI2c {
    base: u64,
    irq: u32,
    idle_lines: u32,
    writable: Vec<u8>,
    clients: BTreeMap<u8, BTreeMap<u8, u8>>,
    regs: [u32; 8],
    address: u8,
    register: u8,
    active: bool,
    reading: bool,
    indexed: bool,
    generation: u64,
    pending: Option<(bool, u8, bool)>,
    transfers: u64,
}

impl RegisterClientsI2c {
    pub fn new(
        base: u64,
        irq: u32,
        idle_lines: u32,
        addresses: Vec<u8>,
        writable: Vec<u8>,
    ) -> Result<Self, String> {
        if base & 3 != 0
            || irq == 0
            || idle_lines & !0x30 != 0
            || addresses.is_empty()
            || writable.is_empty()
            || addresses.iter().any(|address| *address > 0x7f)
        {
            return Err("invalid register-client I2C configuration".into());
        }
        let clients: BTreeMap<_, _> = addresses
            .into_iter()
            .map(|address| (address, BTreeMap::new()))
            .collect();
        if clients.is_empty() {
            return Err("register-client I2C requires clients".into());
        }
        Ok(Self {
            base,
            irq,
            idle_lines,
            writable,
            clients,
            regs: [0; 8],
            address: 0,
            register: 0,
            active: false,
            reading: false,
            indexed: false,
            generation: 0,
            pending: None,
            transfers: 0,
        })
    }

    fn index(&self, address: u64, size: u8) -> Option<usize> {
        let offset = address.checked_sub(self.base)?;
        (matches!(size, 1 | 4) && offset <= 0x1c && offset & 3 == 0).then_some(offset as usize / 4)
    }

    fn transmit(&mut self, cx: &mut Cx<'_>, byte: u8) {
        if !self.indexed {
            if !self.writable.contains(&byte) {
                cx.fail(format!("unmodeled I2C register {byte:#x}"));
                return;
            }
            self.register = byte;
            self.indexed = true;
            return;
        }
        if !self.writable.contains(&self.register) {
            cx.fail(format!("unmodeled I2C register {:#x}", self.register));
            return;
        }
        self.clients
            .get_mut(&self.address)
            .unwrap()
            .insert(self.register, byte);
        self.register = self.register.wrapping_add(1);
    }
}

impl Device for RegisterClientsI2c {
    fn name(&self) -> &'static str {
        "i2c-register-clients"
    }

    fn read(&mut self, cx: &mut Cx<'_>, address: u64, size: u8) -> u64 {
        let Some(index) = self.index(address, size) else {
            cx.unmodeled();
            return 0;
        };
        let value = if index == 7 {
            (self.regs[index] & !0x30) | self.idle_lines
        } else {
            self.regs[index]
        };
        if size == 1 {
            u64::from(value & 0xff)
        } else {
            u64::from(value)
        }
    }

    fn write(&mut self, cx: &mut Cx<'_>, address: u64, size: u8, value: u64) {
        let Some(index) = self.index(address, size) else {
            cx.unmodeled();
            return;
        };
        let value = if size == 1 {
            (self.regs[index] & !0xff) | (value as u32 & 0xff)
        } else {
            value as u32
        };
        self.regs[index] = value;
        if index != 1 {
            return;
        }
        cx.set_irq(self.irq, false);
        self.generation += 1;
        self.pending = None;
        if value & 0x12 != 0x12 {
            self.active = false;
            self.regs[0] = 0;
            return;
        }
        let address_phase = !self.active || value & 0x20 != 0;
        self.pending = Some((address_phase, self.regs[4] as u8, value & 8 != 0));
        cx.schedule(cx.now() + 1_000_000, self.generation);
    }

    fn event(&mut self, cx: &mut Cx<'_>, token: u64) {
        if token != self.generation {
            return;
        }
        let Some((address_phase, byte, ack)) = self.pending.take() else {
            return;
        };
        if address_phase {
            self.address = byte >> 1;
            self.reading = byte & 1 != 0;
            self.active = true;
            if !self.reading {
                self.indexed = false;
            }
            self.regs[0] = if self.reading { 1 } else { 8 };
        } else if self.reading {
            let Some(value) = self
                .clients
                .get(&self.address)
                .and_then(|client| client.get(&self.register))
                .copied()
            else {
                cx.fail(format!(
                    "unmodeled I2C read {:#x}:{:#x}",
                    self.address, self.register
                ));
                return;
            };
            self.regs[4] = u32::from(value);
            self.regs[0] = if ack { 0 } else { 0x10 };
            self.register = self.register.wrapping_add(1);
        } else if self.clients.contains_key(&self.address) {
            self.transmit(cx, byte);
            self.regs[0] = 8;
        }
        if !self.clients.contains_key(&self.address) {
            self.regs[0] |= 0x10;
        }
        self.transfers += 1;
        self.regs[1] |= 0x10;
        if self.regs[1] & 0x40 != 0 {
            cx.set_irq(self.irq, true);
        }
    }

    fn save(&self) -> Value {
        serde_json::to_value(self).unwrap()
    }
    fn restore(&mut self, state: &Value) -> Result<(), String> {
        let saved: Self = serde_json::from_value(state.clone())
            .map_err(|error| format!("invalid register-client I2C checkpoint: {error}"))?;
        if saved.base != self.base
            || saved.irq != self.irq
            || saved.idle_lines != self.idle_lines
            || saved.writable != self.writable
            || saved.clients.keys().collect::<Vec<_>>() != self.clients.keys().collect::<Vec<_>>()
        {
            return Err("register-client I2C checkpoint configuration differs".into());
        }
        *self = saved;
        Ok(())
    }
}

#[cfg(test)]
mod register_client_tests {
    use super::*;
    use fujimod_emulator_devices::test_host::recording::RecordingHost;

    #[test]
    fn selected_client_transfers_signal_and_unknown_client_nacks() {
        let base = 0xff926000;
        let mut device = RegisterClientsI2c::new(base, 290, 0x30, vec![0x0e], vec![0x12]).unwrap();
        let mut host = RecordingHost::default();
        let transfer =
            |device: &mut RegisterClientsI2c, host: &mut RecordingHost, byte: u8, control: u8| {
                let mut cx = Cx::new(host, 0, 0, 0);
                device.write(&mut cx, base + 0x10, 1, u64::from(byte));
                device.write(&mut cx, base + 4, 1, u64::from(control));
                assert!(cx.refusal().is_none());
                let (_, at, token) = host.events.remove(0);
                host.now = at;
                let mut cx = Cx::new(host, 0, 0, 0);
                device.event(&mut cx, token);
                assert!(cx.refusal().is_none());
            };
        transfer(&mut device, &mut host, 0x1c, 0x52);
        assert_eq!(device.regs[0], 8);
        assert_eq!(host.irqs.last(), Some(&(290, true)));
        transfer(&mut device, &mut host, 0x12, 0x12);
        transfer(&mut device, &mut host, 0xff, 0x12);
        assert_eq!(device.clients[&0x0e][&0x12], 0xff);
        let saved = device.save();
        let mut restored =
            RegisterClientsI2c::new(base, 290, 0x30, vec![0x0e], vec![0x12]).unwrap();
        restored.restore(&saved).unwrap();
        assert_eq!(restored.clients[&0x0e][&0x12], 0xff);
        restored.write(&mut Cx::new(&mut host, 0, 0, 0), base + 4, 1, 0);
        transfer(&mut restored, &mut host, 0x1e, 0x52);
        assert_eq!(restored.regs[0] & 0x10, 0x10);
        assert!(RegisterClientsI2c::new(base, 290, 0x30, vec![0x80], vec![0x12]).is_err());
    }
}

#[cfg(test)]
mod idle_tests {
    use super::*;
    use fujimod_emulator_devices::test_host::recording::RecordingHost;

    #[test]
    fn recovery_registers_and_declared_idle_lines_refuse_transfers() {
        let mut device = IdleI2c::new(0xff926000, 0x30).unwrap();
        let mut host = RecordingHost::default();
        let mut cx = Cx::new(&mut host, 0, 0, 0);
        device.write(&mut cx, 0xff926018, 1, 3);
        assert_eq!(device.read(&mut cx, 0xff926018, 1), 3);
        device.write(&mut cx, 0xff92601c, 4, 1);
        assert_eq!(device.read(&mut cx, 0xff92601c, 4), 0x31);
        assert!(cx.refusal().is_none());
        device.write(&mut cx, 0xff926004, 4, 0x12);
        assert!(cx.refusal().is_some());
    }
}
