//! Checked board wiring for serial controllers and their attached parts.
use fujimod_emulator_devices::parts::controls::Controls;
use fujimod_emulator_devices::xp5::pins::SharedPins;
use fujimod_emulator_parts_io::{
    power_controller::PowerController, serial::Endpoint, serial_calibration::CalibrationRegisters,
};
use fujimod_emulator_xp5_io::spi::{Attachment, Select, Spi};
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
enum Part {
    PowerController { rtc_seconds: u32, rtc_valid: bool },
    SerialCalibrationRegisters { miso_high: bool },
    FixedMiso { miso_high: bool, bits: u8, mode: u8 },
    BisConfigurationRegisters { revision: [u8; 2], idle_high: bool },
    Lsm6dsConfigurationRegisters,
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
/// A power controller whose key-ladder registers are driven by the body's
/// controls. Their value is refreshed before every transfer, so the guest reads
/// the held key's ladder value, or the declared released value, and a guest
/// write to one of these registers does not stick.
struct LadderDriven {
    inner: PowerController,
    controls: Arc<dyn Controls>,
    registers: Vec<u8>,
}
impl LadderDriven {
    fn refresh(&mut self) {
        for &register in &self.registers {
            if let Some(value) = self.controls.register_sample(register) {
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
        self.inner.transfer(frame, now_ns)
    }
    fn save(&self) -> serde_json::Value {
        self.inner.save()
    }
    fn restore(&mut self, state: &serde_json::Value) -> Result<(), String> {
        self.inner.restore(state)?;
        self.refresh();
        Ok(())
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
fn endpoint(part: Part, controls: Option<Arc<dyn Controls>>) -> Result<Box<dyn Endpoint>, String> {
    Ok(match part {
        Part::PowerController {
            rtc_seconds,
            rtc_valid,
        } => {
            let mut part = PowerController::new();
            part.configure_rtc(rtc_seconds, rtc_valid);
            match controls
                .map(|controls| (controls.register_ladders(), controls))
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
        Part::BisConfigurationRegisters {
            revision,
            idle_high,
        } => Box::new(
            fujimod_emulator_parts_io::bis_configuration::Registers::new(revision, idle_high)?,
        ),
        Part::Lsm6dsConfigurationRegisters => {
            Box::new(fujimod_emulator_parts_io::lsm6ds::Registers::default())
        }
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
    use fujimod_emulator_parts_controls::ladder_keys::{Ladder, LadderKeys, Options, Rung, Wire};
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
                }],
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
        keys.enqueue(Duration::from_nanos(5), Key::Menu, true)
            .unwrap();
        assert_eq!(read(endpoint.as_mut(), 0x34), 1023, "edge is not due");
        keys.advance(5);
        assert_eq!(read(endpoint.as_mut(), 0x34), 77);
        // Registers the controls do not drive keep the controller's own value.
        assert_eq!(read(endpoint.as_mut(), 0x35), 0);
        keys.enqueue(Duration::from_nanos(9), Key::Menu, false)
            .unwrap();
        keys.advance(9);
        assert_eq!(read(endpoint.as_mut(), 0x34), 1023);
    }

    #[test]
    fn controls_without_register_ladders_leave_the_controller_unwrapped() {
        let controls: Arc<dyn Controls> =
            Arc::new(fujimod_emulator_parts_controls::absent::AbsentControls);
        let mut endpoint = attach(controls);
        assert_eq!(read(endpoint.as_mut(), 0x34), 0);
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
        assert!(fixed.restore(&saved).is_ok());
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
            assert_eq!(other.save(), before);
        }
    }
}
