//! Guest-address translation through the shared Board seam.
//!
//! Every case runs stock-free synthetic data: a four-entry level-1 table, one
//! level-2 table and one level-3 table describe an identity page and a page
//! whose output address differs from its input address.
use super::{
    bus::{Bus, Mode},
    factory, Board,
};
use fujimod_emulator_cpu::{arm64, Access, Register};
use serde_json::json;

const LEVEL1: u64 = 0x1000;
const LEVEL2: u64 = 0x2000;
const LEVEL3: u64 = 0x3000;
const MAPPED_VA: u64 = 0x2000;
const MAPPED_PA: u64 = 0x4000;
const IDENTITY_VA: u64 = 0x10;
const MISSING_VA: u64 = 0x5000;
const DEVICE_VA: u64 = 0x7000;
const DEVICE_PA: u64 = 0x0900_0000;
const TABLE: u64 = 0x403;
const T0SZ_32_BITS: u64 = 0x20;

fn board() -> Board {
    let graph: factory::Graph = serde_json::from_value(json!({
        "processors":[{"model":"armv8a","registers":[],"released":true,"mprofile":null,"rprofile":null}],
        "ram":[{"base":0,"size":65536}],
        "mappings":[{"core":0,"address":0,"physical":0,"size":65536,"permissions":7}],
        "dma_mappings":[{"address":0,"physical":0,"size":65536,"permissions":3}],"decoders":[],"peripherals":[],"instructions_per_tick":[1,1]
    }))
    .unwrap();
    graph
        .build(
            factory::Options::new(factory::Engine::Unicorn)
                .with_translation_cache_size_bytes(1 << 20),
            Bus::new(Mode::Strict),
            None,
        )
        .unwrap()
}

fn words(board: &Board, address: u64, values: &[u64]) {
    let bytes: Vec<u8> = values.iter().flat_map(|value| value.to_le_bytes()).collect();
    board.write_physical(address, &bytes).unwrap();
}

fn register(board: &Board, name: &str) -> Register {
    Register::Arm64System(arm64::system_register(name).unwrap())
}

/// Publish tables that map `0x0` identity and `0x2000` to `0x4000`, then turn
/// translation on for EL1 with a 4 KiB granule and a 32-bit input width.
fn translated_board() -> Board {
    let board = board();
    words(&board, LEVEL1, &[LEVEL2 | 3, 0, 0, 0]);
    words(&board, LEVEL2, &[LEVEL3 | 3]);
    let mut level3 = [0u64; 512];
    level3[0] = TABLE;
    level3[MAPPED_VA as usize >> 12] = MAPPED_PA | TABLE;
    level3[DEVICE_VA as usize >> 12] = DEVICE_PA | TABLE;
    words(&board, LEVEL3, &level3);
    board
        .write_register(0, register(&board, "TTBR0_EL1"), LEVEL1)
        .unwrap();
    board
        .write_register(0, register(&board, "TCR_EL1"), T0SZ_32_BITS)
        .unwrap();
    board
        .write_register(0, register(&board, "SCTLR_EL1"), 1)
        .unwrap();
    board.write_register(0, Register::Pstate, 4).unwrap();
    board
}

#[test]
fn a_nonidentity_page_carries_fetch_load_and_store() {
    let board = translated_board();
    board.write_physical(MAPPED_PA, b"DATA").unwrap();
    let mut bytes = [0u8; 4];
    board.read_guest_memory(0, MAPPED_VA, &mut bytes).unwrap();
    assert_eq!(&bytes, b"DATA");
    assert_eq!(
        board.guest_physical(0, MAPPED_VA, Access::Fetch).unwrap(),
        MAPPED_PA
    );
    board.write_guest_memory(0, MAPPED_VA, b"NEW!").unwrap();
    let mut physical = [0u8; 4];
    board.read_physical(MAPPED_PA, &mut physical).unwrap();
    assert_eq!(&physical, b"NEW!");
}

#[test]
fn an_identity_page_keeps_its_offset() {
    let board = translated_board();
    assert_eq!(
        board.guest_physical(0, IDENTITY_VA, Access::Read).unwrap(),
        IDENTITY_VA
    );
    board.write_guest_memory(0, IDENTITY_VA, b"id").unwrap();
    let mut physical = [0u8; 2];
    board.read_physical(IDENTITY_VA, &mut physical).unwrap();
    assert_eq!(&physical, b"id");
}

#[test]
fn a_missing_descriptor_refuses_at_the_guest_address() {
    let board = translated_board();
    let mut bytes = [0u8; 4];
    let error = board
        .read_guest_memory(0, MISSING_VA, &mut bytes)
        .unwrap_err();
    assert!(error.contains("0x5000"), "{error}");
    assert!(error.contains("invalid AArch64 L3 descriptor"), "{error}");
}

#[test]
fn a_translated_device_window_refuses_instead_of_touching_the_bus() {
    let board = translated_board();
    let error = board
        .guest_physical(0, DEVICE_VA, Access::Read)
        .unwrap_err();
    assert!(error.contains(&format!("{DEVICE_PA:#x}")), "{error}");
    assert!(error.contains("outside modeled RAM"), "{error}");
}

#[test]
fn translation_off_keeps_the_flat_address() {
    let board = board();
    board.write_physical(MAPPED_VA, b"FLAT").unwrap();
    let mut bytes = [0u8; 4];
    board.read_guest_memory(0, MAPPED_VA, &mut bytes).unwrap();
    assert_eq!(&bytes, b"FLAT");
    assert_eq!(
        board.guest_physical(0, MAPPED_VA, Access::Write).unwrap(),
        MAPPED_VA
    );
}

#[test]
fn a_span_crossing_into_an_unmapped_page_refuses() {
    let board = translated_board();
    let mut bytes = [0u8; 8];
    let error = board
        .read_guest_memory(0, MAPPED_VA + 0xffc, &mut bytes)
        .unwrap_err();
    assert!(error.contains("0x3000"), "{error}");
}
