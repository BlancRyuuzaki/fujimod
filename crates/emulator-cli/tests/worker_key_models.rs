//! The worker's declared key model reaches the board: a ladder key edge sent
//! over the JSON-lines protocol changes what the guest reads from the ADC.
use serde_json::{Value, json};
use sha2::{Digest, Sha256};
use std::{
    io::{BufRead, BufReader, Write},
    path::Path,
    process::{Child, ChildStdin, ChildStdout, Command, Stdio},
};

const RAM: u64 = 0x10000;
const ADC: u64 = 0xff90_6000;
const USB_STATUS: u64 = 0xff90_e300;
const RESULT: u64 = 0x11000;
const RELEASED: u16 = 0x456;

/// AArch64 loop: read ADC channel 1 (`+0x14`) and store it at `RESULT`.
fn program() -> Vec<u8> {
    [0xb940_1420u32, 0xb900_0040, 0x17ff_fffe]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect()
}

/// Synthetic ADC ladder: bands and samples are test inputs, not body facts.
fn ladder_keys() -> Value {
    json!({"kind":"ladder-keys","ladders":[{"wire":{"kind":"adc","channel":1},"levels":4096,"rungs":[
        {"key":"Menu","band_low":100,"band_high":200,"sample":150,"basis":"test fixture"},
        {"key":"StickPush","band_low":300,"band_high":400,"sample":350,"basis":"test fixture"}]}]})
}

fn config(directory: &Path, controls: Value) -> Value {
    let program = program();
    std::fs::write(directory.join("program.bin"), &program).unwrap();
    json!({
        "schema": 1, "engine": "unicorn", "mode": "strict",
        "instructions_per_tick": [1, 1], "quantum": 16,
        "ram": [{"base": RAM, "size": 0x4000}],
        "processors": [{"model": "armv8a", "released": true, "mprofile": null, "rprofile": null, "aprofile": null,
            "registers": [{"name":"pc","value":RAM},{"name":"pstate","value":0x3c5},
                {"name":"x1","value":ADC},{"name":"x2","value":RESULT}]}],
        "mappings": [{"core":0,"address":RAM,"physical":RAM,"size":0x4000,"permissions":7}],
        "peripherals": [{"core":0,"address":ADC,"physical":ADC,"size":0x1000}],
        "interrupts": null,
        "devices": {"kind":"xp5","companion":null,"storage":null,"displays":[],
            "windows":[{"kind":"adc","base":ADC,"size":0x1000,"options":{"samples":vec![RELEASED; 15]}}],
            "controls": controls},
        "images": [{"address": RAM, "asset": "program"}],
        "assets": [{"id":"program","content":{"size":program.len(),
            "sha256":format!("{:x}", Sha256::digest(&program))}}],
        "inputs": {"root": directory, "files": {"program": {"kind":"file","path":"program.bin"}},
            "max_asset_bytes": program.len()},
        "output_directory": directory.join("frames"),
    })
}

struct Worker {
    child: Child,
    stdin: ChildStdin,
    stdout: BufReader<ChildStdout>,
}
impl Worker {
    fn spawn() -> Self {
        let mut child = Command::new(env!("CARGO_BIN_EXE_fujimod-emulator-worker"))
            .stdin(Stdio::piped())
            .stdout(Stdio::piped())
            .spawn()
            .unwrap();
        let stdin = child.stdin.take().unwrap();
        let stdout = BufReader::new(child.stdout.take().unwrap());
        Self { child, stdin, stdout }
    }
    fn call(&mut self, request: Value) -> Result<Value, String> {
        writeln!(self.stdin, "{request}").unwrap();
        let mut line = String::new();
        self.stdout.read_line(&mut line).unwrap();
        let reply: Value = serde_json::from_str(&line).unwrap();
        if reply["ok"] == true {
            Ok(reply["result"].clone())
        } else {
            Err(reply["error"].as_str().unwrap().to_owned())
        }
    }
    fn read_sample(&mut self) -> u16 {
        let result = self
            .call(json!({"command":"run-instructions","count":64,"until":null}))
            .unwrap();
        assert!(result["failure"].is_null(), "{result}");
        let hex = self
            .call(json!({"command":"read-memory","core":0,"address":RESULT,"size":2}))
            .unwrap();
        let hex = hex["hex"].as_str().unwrap();
        u16::from_le_bytes([
            u8::from_str_radix(&hex[0..2], 16).unwrap(),
            u8::from_str_radix(&hex[2..4], 16).unwrap(),
        ])
    }
    /// Close the process; a worker that never created a board refuses the
    /// command but still exits.
    fn close(mut self) {
        let _ = self.call(json!({"command":"close"}));
        assert!(self.child.wait().unwrap().success());
    }
}

fn directory(name: &str) -> std::path::PathBuf {
    let directory = Path::new(env!("CARGO_TARGET_TMPDIR")).join(name);
    let _ = std::fs::remove_dir_all(&directory);
    std::fs::create_dir_all(&directory).unwrap();
    directory
}

#[test]
fn ladder_key_edges_reach_the_adc_the_guest_reads_and_undeclared_models_are_refused() {
    let directory = directory("worker-ladder-keys");
    let mut worker = Worker::spawn();
    worker
        .call(json!({"command":"create","config":config(&directory, ladder_keys())}))
        .unwrap();
    assert_eq!(worker.read_sample(), RELEASED);

    worker
        .call(json!({"command":"input","key":"STICK_PUSH","pressed":true,"at":null}))
        .unwrap();
    assert_eq!(worker.read_sample(), 350);
    worker
        .call(json!({"command":"input","key":"STICK_PUSH","pressed":false,"at":null}))
        .unwrap();
    assert_eq!(worker.read_sample(), RELEASED);
    worker
        .call(json!({"command":"input","key":"MENU","pressed":true,"at":null}))
        .unwrap();
    assert_eq!(worker.read_sample(), 150);

    // A key without a rung has no electrical description in this model.
    let error = worker
        .call(json!({"command":"input","key":"DISP/BACK","pressed":true,"at":null}))
        .unwrap_err();
    assert!(error.contains("no declared ladder rung"), "{error}");
    worker.close();

    // Key models are named by mechanism; a camera name is not one. One worker
    // at a time, so this shares the test above.
    let mut worker = Worker::spawn();
    let mut controls = ladder_keys();
    controls["kind"] = json!("x100vi");
    let error = worker
        .call(json!({"command":"create","config":config(&directory, controls)}))
        .unwrap_err();
    assert!(error.contains("unknown variant"), "{error}");
    worker.close();
}

#[test]
fn core_reset_releases_an_arm_core_without_patching_guest_code() {
    let directory = directory("worker-core-reset");
    let mut configuration = config(&directory, ladder_keys());
    let mut held_processor = configuration["processors"][0].clone();
    held_processor["released"] = json!(false);
    configuration["processors"].as_array_mut().unwrap().push(held_processor);
    let mut mapping = configuration["mappings"][0].clone();
    mapping["core"] = json!(1);
    configuration["mappings"].as_array_mut().unwrap().push(mapping);
    let mut peripheral = configuration["peripherals"][0].clone();
    peripheral["core"] = json!(1);
    configuration["peripherals"].as_array_mut().unwrap().push(peripheral);
    let mut worker = Worker::spawn();
    worker
        .call(json!({"command":"create","config":configuration}))
        .unwrap();

    let code = worker
        .call(json!({"command":"read-memory","core":0,"address":RAM,"size":12}))
        .unwrap();
    let result = worker
        .call(json!({"command":"run-instructions","count":64,"until":null}))
        .unwrap();
    assert_eq!(result["instructions"][1], 0);
    assert!(result["failure"].is_null(), "{result}");

    assert_eq!(
        worker
            .call(json!({"command":"core-reset","core":1,"asserted":false}))
            .unwrap(),
        Value::Null
    );
    let result = worker
        .call(json!({"command":"run-instructions","count":1,"until":null}))
        .unwrap();
    assert!(result["instructions"][1].as_u64().unwrap() > 0, "{result}");
    assert!(result["failure"].is_null(), "{result}");
    assert_eq!(
        worker
            .call(json!({"command":"read-memory","core":0,"address":RAM,"size":12}))
            .unwrap(),
        code
    );
    worker.close();
}

#[test]
fn memory_write_watch_commands_capture_bounded_cpu_stores_and_end_cleanly() {
    let directory = directory("worker-memory-write-watch");
    let mut worker = Worker::spawn();
    worker
        .call(json!({"command":"create","config":config(&directory, ladder_keys())}))
        .unwrap();

    worker
        .call(json!({"command":"memory-write-watch-begin","ranges":[[RESULT,RESULT + 2]],"limit":8}))
        .unwrap();
    let result = worker
        .call(json!({"command":"run-instructions","count":3,"until":null}))
        .unwrap();
    assert!(result["failure"].is_null(), "{result}");
    let state = worker
        .call(json!({"command":"memory-write-watch-state"}))
        .unwrap();
    assert_eq!(state["enabled"], true);
    assert_eq!(state["records"].as_array().unwrap().len(), 1, "{state}");
    assert_eq!(state["records"][0]["physical"], RESULT);
    assert_eq!(
        worker
            .call(json!({"command":"memory-write-watch-end"}))
            .unwrap(),
        state
    );
    assert!(worker
        .call(json!({"command":"memory-write-watch-state"}))
        .unwrap_err()
        .contains("No memory write watch is active"));
    worker.close();
}

fn usb_config(directory: &Path, include_usb_status: bool) -> Value {
    let program = [0xb940_0020u32, 0xb900_0040, 0x17ff_fffe]
        .iter()
        .flat_map(|word| word.to_le_bytes())
        .collect::<Vec<_>>();
    std::fs::write(directory.join("program.bin"), &program).unwrap();
    let mut windows = vec![
        json!({"kind":"adc","base":ADC,"size":0x1000,"options":{"samples":vec![RELEASED; 15]}}),
    ];
    let mut peripherals = vec![json!({"core":0,"address":ADC,"physical":ADC,"size":0x1000})];
    if include_usb_status {
        windows
            .push(json!({"kind":"usb-connection-status","base":USB_STATUS,"size":4,"options":{}}));
        peripherals.push(json!({"core":0,"address":USB_STATUS,"physical":USB_STATUS,"size":4}));
    }
    json!({
        "schema": 1, "engine": "unicorn", "mode": "strict",
        "instructions_per_tick": [1, 1], "quantum": 16,
        "ram": [{"base": RAM, "size": 0x4000}],
        "processors": [{"model": "armv8a", "released": true, "mprofile": null, "rprofile": null, "aprofile": null,
            "registers": [{"name":"pc","value":RAM},{"name":"pstate","value":0x3c5},
                {"name":"x1","value":USB_STATUS},{"name":"x2","value":RESULT}]}],
        "mappings": [{"core":0,"address":RAM,"physical":RAM,"size":0x4000,"permissions":7}],
        "peripherals": peripherals,
        "interrupts": null,
        "devices": {"kind":"xp5","companion":null,"storage":null,"displays":[],
            "windows": windows,
            "controls": ladder_keys()},
        "images": [{"address": RAM, "asset": "program"}],
        "assets": [{"id":"program","content":{"size":program.len(),
            "sha256":format!("{:x}", Sha256::digest(&program))}}],
        "inputs": {"root": directory, "files": {"program": {"kind":"file","path":"program.bin"}},
            "max_asset_bytes": program.len()},
        "output_directory": directory.join("frames"),
    })
}

#[test]
fn usb_connection_command_changes_checked_status_device_and_refuses_missing_node() {
    let directory = directory("worker-usb-connection");
    let mut worker = Worker::spawn();
    worker
        .call(json!({"command":"create","config":usb_config(&directory, true)}))
        .unwrap();

    let read_status = |worker: &mut Worker| {
        let result = worker
            .call(json!({"command":"run-instructions","count":3,"until":null}))
            .unwrap();
        assert!(result["failure"].is_null(), "{result}");
        let memory = worker
            .call(json!({"command":"read-memory","core":0,"address":RESULT,"size":4}))
            .unwrap();
        u32::from_le_bytes(
            (0..4)
                .map(|i| {
                    u8::from_str_radix(&memory["hex"].as_str().unwrap()[i * 2..i * 2 + 2], 16)
                        .unwrap()
                })
                .collect::<Vec<_>>()
                .try_into()
                .unwrap(),
        )
    };
    assert_eq!(read_status(&mut worker), 0);
    worker
        .call(json!({"command":"usb-connection","base":USB_STATUS,"connected":true}))
        .unwrap();
    assert_eq!(read_status(&mut worker), 1);
    worker.close();

    let mut worker = Worker::spawn();
    worker
        .call(json!({"command":"create","config":usb_config(&directory, false)}))
        .unwrap();
    let error = worker
        .call(json!({"command":"usb-connection","base":USB_STATUS,"connected":true}))
        .unwrap_err();
    assert!(
        error.contains("no declared usb-connection-status"),
        "{error}"
    );
    worker.close();
}
