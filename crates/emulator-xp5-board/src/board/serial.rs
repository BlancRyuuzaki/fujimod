//! Checked board wiring for serial controllers and their attached parts.
use fujimod_emulator_parts_io::{power_controller::PowerController, serial::Endpoint, serial_calibration::CalibrationRegisters};
use fujimod_emulator_devices::xp5::pins::SharedPins;
use fujimod_emulator_xp5_io::spi::{Attachment, Select, Spi};
use fujimod_emulator_devices::parts::controls::Controls;
use serde::Deserialize;
use std::sync::Arc;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Wiring {
    source_hz: u64,
    endpoints: Vec<Connection>,
}
#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
pub(super) struct Connection {
    select: Select,
    part: Part,
}
#[derive(Deserialize)]
#[serde(tag = "kind", rename_all = "kebab-case", deny_unknown_fields)]
pub(super) enum Part {
    PowerController {
        rtc_seconds: u32,
        rtc_valid: bool,
        /// Bring-up: initial register overrides (address -> value), for
        /// example register 0 bit 7, which the X-T5 stage1 polls at boot.
        #[serde(default)]
        registers: std::collections::BTreeMap<String, u16>,
    },
    SerialCalibrationRegisters { miso_high: bool },
    FixedMiso { miso_high: bool, bits: u8, mode: u8 },
    SerialCommandRegisters(fujimod_emulator_parts_io::serial_command_registers::Wiring),
    BisConfigurationRegisters {
        revision: [u8; 2],
        samples: [u16; 2],
        idle_high: bool,
        #[serde(default)]
        block: Option<[u8; 6]>,
    },
    Lsm6dsConfigurationRegisters {
        temperature: i16,
        #[serde(default)]
        motion: Option<fujimod_emulator_parts_io::lsm6ds::Motion>,
    },
    FramedCommandPeer { initial_frame: u32 },
    /// Bring-up: an unidentified chip. Accepts every frame, records the
    /// last words written and answers with a fixed MISO level.
    GenericSink { miso_high: bool },
}
/// A fixed electrical MISO input with an explicitly declared serial frame.
/// Transmitted words do not select or influence the returned level.
struct FixedMiso {
    miso_high: bool,
    bits: u8,
    mode: u8,
    transfers: u64,
}
impl FixedMiso {
    fn new(miso_high: bool, bits: u8, mode: u8) -> Result<Self, String> {
        if !(4..=16).contains(&bits) || mode > 3 {
            return Err("fixed MISO requires a 4- to 16-bit frame and Motorola mode 0-3".into());
        }
        Ok(Self {
            miso_high,
            bits,
            mode,
            transfers: 0,
        })
    }
}
impl Endpoint for FixedMiso {
    fn name(&self) -> &'static str {
        "fixed-miso"
    }
    fn validate(&self, frame: fujimod_emulator_parts_io::serial::Frame) -> Result<(), String> {
        if frame.bits != self.bits || frame.mode != self.mode {
            return Err(format!(
                "fixed MISO requires a {}-bit mode{} frame",
                self.bits, self.mode
            ));
        }
        Ok(())
    }
    fn reset_transaction(&mut self) {}
    fn transfer(&mut self, _: fujimod_emulator_parts_io::serial::Frame, _: u64) -> u16 {
        self.transfers += 1;
        if self.miso_high {
            u16::MAX >> (16 - self.bits)
        } else {
            0
        }
    }
    fn checkpoint(&self) -> Result<fujimod_emulator_devices::checkpoint::Checkpoint, String> {
        fujimod_emulator_devices::checkpoint::Checkpoint::encode(
            self.name(),
            &(self.miso_high, self.bits, self.mode, self.transfers),
        )
    }
    fn validate_checkpoint(
        &self,
        saved: &fujimod_emulator_devices::checkpoint::Checkpoint,
    ) -> Result<(), String> {
        let (miso_high, bits, mode, _): (bool, u8, u8, u64) = saved.decode(self.name())?;
        if (miso_high, bits, mode) != (self.miso_high, self.bits, self.mode) {
            return Err("fixed MISO configuration differs from checkpoint".into());
        }
        Ok(())
    }
    fn restore_checkpoint(
        &mut self,
        saved: &fujimod_emulator_devices::checkpoint::Checkpoint,
    ) -> Result<(), String> {
        self.validate_checkpoint(saved)?;
        let (_, _, _, transfers): (bool, u8, u8, u64) = saved.decode(self.name())?;
        self.transfers = transfers;
        Ok(())
    }
    fn save(&self) -> serde_json::Value {
        serde_json::json!({"miso_high":self.miso_high, "bits":self.bits, "mode":self.mode, "transfers":self.transfers})
    }
    fn restore(&mut self, state: &serde_json::Value) -> Result<(), String> {
        if state["miso_high"].as_bool() != Some(self.miso_high)
            || state["bits"].as_u64() != Some(self.bits.into())
            || state["mode"].as_u64() != Some(self.mode.into())
        {
            return Err("fixed MISO configuration differs from checkpoint".into());
        }
        let transfers = state["transfers"]
            .as_u64()
            .ok_or("invalid fixed MISO transfer count")?;
        self.transfers = transfers;
        Ok(())
    }
}
/// Bring-up: endpoint for bring-up of bodies whose serial parts are not yet identified.
struct GenericSink {
    miso_high: bool,
    transfers: u64,
    last: Vec<u16>,
}
impl Endpoint for GenericSink {
    fn name(&self) -> &'static str {
        "generic-sink"
    }
    fn validate(&self, _: fujimod_emulator_parts_io::serial::Frame) -> Result<(), String> {
        Ok(())
    }
    fn reset_transaction(&mut self) {}
    fn transfer(&mut self, frame: fujimod_emulator_parts_io::serial::Frame, _: u64) -> u16 {
        self.transfers += 1;
        self.last.push(frame.word);
        if self.last.len() > 16 {
            self.last.remove(0);
        }
        if self.miso_high { u16::MAX } else { 0 }
    }
    fujimod_emulator_devices::checkpoint_fields!(miso_high: bool, transfers: u64, last: Vec<u16>);
    fn save(&self) -> serde_json::Value {
        serde_json::json!({"miso_high": self.miso_high, "transfers": self.transfers, "last": self.last})
    }
    fn restore(&mut self, state: &serde_json::Value) -> Result<(), String> {
        self.transfers = state["transfers"].as_u64().unwrap_or(0);
        self.last = state["last"].as_array().map(|a| a.iter().filter_map(|v| v.as_u64().map(|v| v as u16)).collect()).unwrap_or_default();
        Ok(())
    }
}
/// A power controller whose key-ladder registers, switch bits and dial count
/// are driven by the body's controls. Ladder and switch registers are
/// refreshed before every transfer, so the guest reads the held keys' ladder
/// value, or the declared released value, and a guest write to them does not
/// stick. A counting register is loaded when the guest starts to read it, and
/// that read consumes the count it returns.
struct LadderDriven {
    inner: PowerController,
    controls: Arc<dyn Controls>,
    registers: Vec<u8>,
}
impl LadderDriven {
    fn refresh(&mut self) {
        for &register in &self.registers {
            if let Some(value) = self.controls.drive_register(register, self.inner.register(register)) {
                self.inner.set_register(register, value);
            }
        }
    }
}
impl Endpoint for LadderDriven {
    fn name(&self) -> &'static str {
        self.inner.name()
    }
    fn validate(&self, frame: fujimod_emulator_parts_io::serial::Frame) -> Result<(), String> {
        self.inner.validate(frame)
    }
    fn reset_transaction(&mut self) {
        self.inner.reset_transaction()
    }
    fn transfer(&mut self, frame: fujimod_emulator_parts_io::serial::Frame, now_ns: u64) -> u16 {
        self.refresh();
        let command = frame.word as u8;
        if self.inner.transfer_position() == 0
            && command & 0x80 == 0
            && let Some(count) = self.controls.take_register(command & 0x7f)
        {
            self.inner.set_register(command & 0x7f, count);
        }
        self.inner.transfer(frame, now_ns)
    }
    fn checkpoint(&self) -> Result<fujimod_emulator_devices::checkpoint::Checkpoint, String> {
        self.inner.checkpoint()
    }
    fn validate_checkpoint(&self, saved: &fujimod_emulator_devices::checkpoint::Checkpoint) -> Result<(), String> {
        self.inner.validate_checkpoint(saved)
    }
    fn restore_checkpoint(&mut self, saved: &fujimod_emulator_devices::checkpoint::Checkpoint) -> Result<(), String> {
        self.inner.restore_checkpoint(saved)
    }
    fn save(&self) -> serde_json::Value {
        self.inner.save()
    }
    fn restore(&mut self, state: &serde_json::Value) -> Result<(), String> {
        self.inner.restore(state)?;
        self.refresh();
        Ok(())
    }
    fn set_input_bits(&mut self, register: u8, mask: u16, value: u16) -> Result<(), String> {
        if self.registers.contains(&register) {
            return Err("power-controller register is driven by the body controls".into());
        }
        Endpoint::set_input_bits(&mut self.inner, register, mask, value)
    }
}

pub fn build(
    base: u64,
    pins: SharedPins,
    options: &toml::Table,
    controls: Arc<dyn Controls>,
) -> Result<Spi, String> {
    let wiring: Wiring = toml::Value::Table(options.clone())
        .try_into()
        .map_err(|e| format!("SPI wiring: {e}"))?;
    Spi::new(
        base,
        pins,
        wiring.source_hz,
        attachments(wiring.endpoints, Some(controls))?,
    )
}
/// Parse `XT5_POWER_REGISTERS="REGISTER=VALUE,..."` (numbers decimal or `0x` hexadecimal):
/// power-controller registers a run sets at power-on in place of the part's defaults. A host input
/// like `XT5_ADC`, kept outside the manifest. On the X-T5, register 0x32 is the battery
/// temperature in 0.1 K (2982 = 25.0 C); it reads 0 with no battery data, and the battery driver
/// then reports external power.
fn parse_power_registers(spec: &str) -> Result<Vec<(u8, u16)>, String> {
    fn number(text: &str) -> Option<u32> {
        let text = text.trim();
        match text.strip_prefix("0x") {
            Some(hex) => u32::from_str_radix(hex, 16).ok(),
            None => text.parse().ok(),
        }
    }
    spec.split(',')
        .map(str::trim)
        .filter(|item| !item.is_empty())
        .map(|item| {
            let (register, value) =
                item.split_once('=').ok_or("XT5_POWER_REGISTERS needs REGISTER=VALUE[,...]")?;
            let register = number(register)
                .filter(|register| *register < 0x80)
                .ok_or_else(|| format!("XT5_POWER_REGISTERS: invalid register in {item}"))?;
            let value = number(value)
                .and_then(|value| u16::try_from(value).ok())
                .ok_or_else(|| format!("XT5_POWER_REGISTERS: invalid value in {item}"))?;
            Ok((register as u8, value))
        })
        .collect()
}
fn endpoint(part: Part, controls: Option<Arc<dyn Controls>>) -> Result<Box<dyn Endpoint>, String> {
    Ok(match part {
        Part::PowerController {
            rtc_seconds,
            rtc_valid,
            registers,
        } => {
            let mut part = PowerController::new();
            part.configure_rtc(rtc_seconds, rtc_valid);
            // Each address appears once and names a plain storage register; the
            // RTC latch is refused.
            let mut seen = [false; 0x80];
            for (address, value) in registers {
                let address = u8::from_str_radix(address.trim_start_matches("0x"), 16)
                    .map_err(|e| format!("power-controller register {address}: {e}"))?;
                if address >= 0x80 || address == 0x27 {
                    return Err(format!(
                        "power controller register {address:#x} is not a declarable storage register"
                    ));
                }
                if std::mem::replace(&mut seen[address as usize], true) {
                    return Err(format!(
                        "power controller register {address:#x} is declared twice"
                    ));
                }
                part.set_register(address, value);
            }
            if let Ok(spec) = std::env::var("XT5_POWER_REGISTERS") {
                for (address, value) in parse_power_registers(&spec)? {
                    part.set_register(address, value);
                }
            }
            match controls
                .map(|controls| (controls.driven_registers(), controls))
                .filter(|(registers, _)| !registers.is_empty())
            {
                Some((registers, controls)) => {
                    let mut driven = LadderDriven {
                        inner: part,
                        controls,
                        registers,
                    };
                    driven.refresh();
                    Box::new(driven)
                }
                None => Box::new(part),
            }
        }
        Part::SerialCalibrationRegisters { miso_high } => {
            Box::new(CalibrationRegisters::new(miso_high))
        }
        Part::FixedMiso {
            miso_high,
            bits,
            mode,
        } => Box::new(FixedMiso::new(miso_high, bits, mode)?),
        Part::SerialCommandRegisters(wiring) => Box::new(
            fujimod_emulator_parts_io::serial_command_registers::CommandRegisters::new(wiring)?,
        ),
        Part::BisConfigurationRegisters {
            revision,
            samples,
            idle_high,
            block,
        } => Box::new(fujimod_emulator_parts_io::bis_configuration::Registers::with_block(
            revision, samples, idle_high, block,
        )?),
        Part::Lsm6dsConfigurationRegisters { temperature, motion } => {
            Box::new(fujimod_emulator_parts_io::lsm6ds::Registers::new(temperature, motion))
        }
        Part::FramedCommandPeer { initial_frame } => {
            Box::new(fujimod_emulator_parts_io::framed_command_peer::Peer::new(initial_frame)?)
        }
        Part::GenericSink { miso_high } => Box::new(GenericSink { miso_high, transfers: 0, last: vec![] }),
    })
}
fn attachments(
    connections: Vec<Connection>,
    controls: Option<Arc<dyn Controls>>,
) -> Result<Vec<Attachment>, String> {
    connections
        .into_iter()
        .map(|connection| {
            Ok(Attachment::new(
                connection.select,
                endpoint(connection.part, controls.clone())?,
            ))
        })
        .collect()
}
pub(super) fn dedicated_wire(
    part: Part,
) -> Result<Box<dyn fujimod_emulator_parts_io::serial::Wire>, String> {
    Ok(Box::new(fujimod_emulator_parts_io::serial::Dedicated(endpoint(part, None)?)))
}
pub(super) fn gpio_wire(
    pins: SharedPins,
    connections: Vec<Connection>,
) -> Result<Box<dyn fujimod_emulator_parts_io::serial::Wire>, String> {
    Ok(Box::new(fujimod_emulator_xp5_io::spi::GpioWire::new(
        pins,
        attachments(connections, None)?,
    )?))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn power_register_inputs_parse_decimal_and_hexadecimal() {
        assert_eq!(parse_power_registers("0x32=2982, 3=600").unwrap(), [(0x32, 2982), (3, 600)]);
        assert_eq!(parse_power_registers("").unwrap(), []);
        assert!(parse_power_registers("0x32").unwrap_err().contains("REGISTER=VALUE"));
        assert!(parse_power_registers("0x80=1").unwrap_err().contains("invalid register"));
        assert!(parse_power_registers("3=0x10000").unwrap_err().contains("invalid value"));
    }
    use fujimod_emulator_parts_controls::ladder_keys::{CountingDial, Ladder, LadderKeys, Options, RegisterSwitch, Rung, Wire};
    use fujimod_emulator_parts_controls::resistor_keys::Key;
    use fujimod_emulator_parts_io::serial::Frame;
    use std::time::Duration;

    // Synthetic fixture: bands and samples are test inputs, not camera values.
    fn keys() -> Arc<LadderKeys> {
        let rung = |key, sample| Rung {
            key,
            band_low: sample,
            band_high: sample,
            sample,
            basis: "test fixture".into(),
        };
        Arc::new(
            LadderKeys::new(Options {
                ladders: vec![Ladder {
                    wire: Wire::PowerControllerRegister {
                        register: 0x34,
                        released: 1023,
                    },
                    levels: 1024,
                    rungs: vec![rung(Key::Menu, 77)],
                    sets: vec![],
                }],
                dials: vec![],
                switches: vec![RegisterSwitch {
                    name: "power".into(),
                    register: 0,
                    bit: 10,
                    set: "OFF".into(),
                    clear: "ON".into(),
                    basis: "test fixture".into(),
                }],
                rear_dial: Some(CountingDial { register: 0x36, basis: "test fixture".into() }),
                front_dial: None,
                gpio_keys: vec![],
            })
            .unwrap(),
        )
    }

    fn read(endpoint: &mut dyn Endpoint, register: u8) -> u16 {
        let request = [register, 0, 0, !register];
        let reply: Vec<u8> = request
            .into_iter()
            .map(|byte| {
                endpoint.transfer(
                    Frame {
                        bits: 8,
                        mode: 3,
                        word: byte.into(),
                    },
                    0,
                ) as u8
            })
            .collect();
        u16::from_be_bytes([reply[1], reply[2]])
    }

    fn attach(controls: Arc<dyn Controls>) -> Box<dyn Endpoint> {
        endpoint(
            Part::PowerController {
                registers: Default::default(),
                rtc_seconds: 0,
                rtc_valid: true,
            },
            Some(controls),
        )
        .unwrap()
    }

    #[test]
    fn declared_register_ladders_reach_the_power_controller_registers() {
        let keys = keys();
        let mut endpoint = attach(keys.clone());
        assert_eq!(read(endpoint.as_mut(), 0x34), 1023);
        keys.enqueue(Duration::from_nanos(5), Key::Menu, true).unwrap();
        assert_eq!(read(endpoint.as_mut(), 0x34), 1023, "edge is not due");
        keys.advance(5);
        assert_eq!(read(endpoint.as_mut(), 0x34), 77);
        // Registers the controls do not drive keep the controller's own value.
        assert_eq!(read(endpoint.as_mut(), 0x35), 0);
        keys.enqueue(Duration::from_nanos(9), Key::Menu, false).unwrap();
        keys.advance(9);
        assert_eq!(read(endpoint.as_mut(), 0x34), 1023);
    }

    #[test]
    fn switch_bits_and_the_dial_count_reach_the_power_controller() {
        let keys = keys();
        let mut endpoint = attach(keys.clone());
        assert_eq!(read(endpoint.as_mut(), 0x00), 0x200, "no position: reset value");
        assert_eq!(read(endpoint.as_mut(), 0x36), 0);
        keys.set_position(Duration::from_nanos(3), "power", "OFF").unwrap();
        for _ in 0..3 {
            keys.enqueue(Duration::from_nanos(3), Key::RearMinus, true).unwrap();
        }
        keys.advance(3);
        assert_eq!(read(endpoint.as_mut(), 0x00), 0x600);
        // -3 as a signed nibble, then cleared by that read.
        assert_eq!(read(endpoint.as_mut(), 0x36), 0xd);
        assert_eq!(read(endpoint.as_mut(), 0x36), 0);
        keys.set_position(Duration::from_nanos(4), "power", "ON").unwrap();
        keys.advance(4);
        assert_eq!(read(endpoint.as_mut(), 0x00), 0x200);
    }

    #[test]
    fn command_register_endpoints_take_their_declared_wire_format() {
        let connection = |part: &str| {
            toml::from_str::<Connection>(&format!(
                "select = {{ port = 12, pin = 8, active_high = false }}\npart = {part}"
            ))
        };
        let burst = connection(
            "{ kind = 'serial-command-registers', bits = 8, mode = 0, read_flag = 1, miso_high = false }",
        )
        .unwrap();
        let mut part = endpoint(burst.part, None).unwrap();
        for word in [0x0a, 0x11, 0x22] {
            let frame = Frame { bits: 8, mode: 0, word };
            part.validate(frame).unwrap();
            part.transfer(frame, 0);
        }
        assert_eq!(part.save()["registers"]["registers"]["10"], serde_json::json!([0x11, 0x22]));
        assert!(connection("{ kind = 'serial-command-registers', mode = 0, miso_high = false }").is_err());
        assert!(
            connection(
                "{ kind = 'serial-command-registers', bits = 8, mode = 0, read_flag = 1, miso_high = false, idle = 1 }"
            )
            .is_err()
        );
    }

    #[test]
    fn controls_without_register_ladders_leave_the_controller_unwrapped() {
        let controls: Arc<dyn Controls> =
            Arc::new(fujimod_emulator_parts_controls::absent::AbsentControls);
        let mut endpoint = attach(controls);
        assert_eq!(read(endpoint.as_mut(), 0x34), 0);
    }

    #[test]
    fn declared_registers_override_reset_values() {
        let power_controller = |registers: &str| {
            endpoint(
                Part::PowerController {
                    rtc_seconds: 0,
                    rtc_valid: false,
                    registers: toml::from_str(registers).unwrap(),
                },
                None,
            )
        };
        let mut default_part = power_controller("").unwrap();
        assert_eq!(read(default_part.as_mut(), 0), 0x200, "stock default");
        assert_eq!(read(default_part.as_mut(), 0) & 0x8000, 0);

        let mut ready_part = power_controller("\"0\" = 0x8200\n\"0x10\" = 2").unwrap();
        assert_eq!(read(ready_part.as_mut(), 0), 0x8200, "bit-15 candidate");
        assert_eq!(read(ready_part.as_mut(), 0x10), 2);
        for registers in [
            "\"27\" = 0x200",
            "\"80\" = 0",
            "\"10\" = 1\n\"0x10\" = 2",
        ] {
            assert!(power_controller(registers).is_err(), "{registers}");
        }
    }

    #[test]
    fn fixed_miso_returns_the_selected_level_for_matching_frames() {
        for (miso_high, expected) in [(true, 0x0fff), (false, 0)] {
            let mut endpoint = endpoint(
                Part::FixedMiso {
                    miso_high,
                    bits: 12,
                    mode: 2,
                },
                None,
            )
            .unwrap();
            let frame = Frame {
                bits: 12,
                mode: 2,
                word: 0xabc,
            };
            endpoint.validate(frame).unwrap();
            assert_eq!(endpoint.transfer(frame, 0), expected);
        }
    }

    #[test]
    fn fixed_miso_refuses_wrong_frame_configuration_without_mutation() {
        let fixed = endpoint(
            Part::FixedMiso {
                miso_high: true,
                bits: 12,
                mode: 2,
            },
            None,
        )
        .unwrap();
        let before = fixed.save();
        for frame in [
            Frame {
                bits: 11,
                mode: 2,
                word: 0xabc,
            },
            Frame {
                bits: 12,
                mode: 1,
                word: 0xabc,
            },
        ] {
            assert!(fixed.validate(frame).is_err());
            assert_eq!(fixed.save(), before);
        }
    }

    #[test]
    fn fixed_miso_restore_checks_its_external_configuration() {
        let mut fixed = endpoint(
            Part::FixedMiso {
                miso_high: true,
                bits: 12,
                mode: 2,
            },
            None,
        )
        .unwrap();
        let saved = fixed.save();
        let exact = fixed.checkpoint().unwrap();
        assert!(fixed.restore(&saved).is_ok());
        fixed.restore_checkpoint(&exact).unwrap();
        for other in [
            Part::FixedMiso {
                miso_high: false,
                bits: 12,
                mode: 2,
            },
            Part::FixedMiso {
                miso_high: true,
                bits: 8,
                mode: 2,
            },
            Part::FixedMiso {
                miso_high: true,
                bits: 12,
                mode: 0,
            },
        ] {
            let mut other = endpoint(other, None).unwrap();
            let before = other.save();
            assert!(other.restore(&saved).is_err());
            assert!(other.validate_checkpoint(&exact).is_err());
            assert_eq!(other.save(), before);
        }
    }
}
