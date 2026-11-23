//! Keeping an M-profile processor's translations across a peer's slice must
//! run exactly the instructions that discarding them on every switch runs.
use super::*;
use fujimod_catalog_configuration::RunEngine;
use fujimod_emulator_cpu::Model;

/// A Cortex-M4 loop in RAM that a Cortex-R5F peer later patches through its
/// own view of the same RAM. The patch replaces `adds r0,#1` with
/// `adds r1,#1`, so R0 freezes at the M4 instruction where the store landed.
fn run(quantum: usize, retain: bool, patch: u32) -> Outcome {
    run_with(quantum, retain, patch, None, ExecutionPolicy::Exact, false)
}

/// The board state, R0 and R1 (R2 after a host write), and the shadow's kept
/// entries and discards.
type Outcome = (Value, [u64; 2], u64, u64);

/// As `run`, with an optional host write of `adds r2,#1` over the M4 loop
/// between two run windows, the way a timed patch or a poke lands. With
/// `stores`, each round also calls a routine in the next page that stores R0
/// back into the loop's page, so the M4 writes code it is not running.
fn run_with(
    quantum: usize,
    retain: bool,
    patch: u32,
    host_write_at: Option<u64>,
    policy: ExecutionPolicy,
    stores: bool,
) -> Outcome {
    use factory::*;
    let graph = Graph {
        decoders: vec![],
        processors: vec![
            Processor {
                model: Model::CortexM4,
                registers: vec![],
                released: true, parked_until_sgi: false,
                rprofile: None,
                mprofile: Some(super::mprofile::Config {
                    clock_hz: 1_000_000_000,
                    board_ticks_per_second: 1_000_000_000,
                    vector_base: 0,
                }),
            },
            Processor {
                rprofile: Some(Default::default()),
                mprofile: None,
                model: Model::CortexR5F,
                registers: vec![(Register::Pc, 0x10000)],
                released: true, parked_until_sgi: false,
            },
        ],
        dma_mappings: vec![
            crate::board::memory::BusMapping::ram(0, 0x4000),
            crate::board::memory::BusMapping::ram(0x10000, 0x4000),
        ],
        ram: vec![
            Ram {
                base: 0,
                size: 0x4000,
            },
            Ram {
                base: 0x10000,
                size: 0x4000,
            },
        ],
        mappings: vec![
            MemoryMap {
                core: 0,
                address: 0,
                physical: 0,
                size: 0x4000,
                permissions: 7,
            },
            MemoryMap {
                core: 1,
                address: 0x10000,
                physical: 0x10000,
                size: 0x4000,
                permissions: 7,
            },
            MemoryMap {
                core: 1,
                address: 0x20000,
                physical: 0,
                size: 0x4000,
                permissions: 3,
            },
        ],
        peripherals: vec![],
        instructions_per_tick: Ratio::new(1, 1).unwrap(),
    };
    let board = graph
        .build(
            Options::new(RunEngine::Unicorn).with_translation_cache_size_bytes(1024 * 1024),
            Bus::new(bus::Mode::Strict),
            None,
        )
        .unwrap();
    let mut board = board;
    board.set_execution_policy(policy).unwrap();
    if !retain {
        board.state.borrow_mut().code_shadow[0] = None;
    }
    let m4 = board.cpu(0).unwrap();
    let entry = if stores { 0x1011u32 } else { 0x1001 };
    m4.write_memory(
        0,
        &[0x3000u32, entry]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect::<Vec<_>>(),
    )
    .unwrap();
    if stores {
        // 0x1000: adds r0,#1; bl 0x2000; b 0x1000
        // 0x1010: movs r6,#0x11; lsls r6,r6,#8; b 0x1000
        // 0x2000: str r0,[r6]; bx lr
        m4.write_memory(0x1000, &[0x01, 0x30, 0x00, 0xf0, 0xfd, 0xff, 0xfb, 0xe7])
            .unwrap();
        m4.write_memory(0x1010, &[0x11, 0x26, 0x36, 0x02, 0xf4, 0xe7])
            .unwrap();
        m4.write_memory(0x2000, &[0x30, 0x60, 0x70, 0x47]).unwrap();
    } else {
        // adds r0,#1; b .-2
        m4.write_memory(0x1000, &[0x01, 0x30, 0xfd, 0xe7]).unwrap();
    }
    // mov r3,#40; subs r3,r3,#1; bne .-4; mov r4,#0x21000; mov r5,#PATCH_HIGH;
    // orr r5,r5,#PATCH_LOW; strh r5,[r4]; b .
    let peer = [
        0xe3a03028u32,
        0xe2533001,
        0x1afffffd,
        0xe3a04a21,
        0xe3a05c00 | (patch >> 8),
        0xe3855000 | (patch & 0xff),
        0xe1c450b0,
        0xeafffffe,
    ];
    board
        .cpu(1)
        .unwrap()
        .write_memory(
            0x10000,
            &peer.into_iter().flat_map(u32::to_le_bytes).collect::<Vec<_>>(),
        )
        .unwrap();
    if let Some(at) = host_write_at {
        board.run_until(at, quantum).unwrap();
        board.write_physical(0x1000, &[0x01, 0x32]).unwrap();
    }
    board.run_until(400, quantum).unwrap();
    let m4 = board.cpu(0).unwrap();
    let registers = [0, if host_write_at.is_some() { 2 } else { 1 }].map(|r| {
        scalar(m4, Register::Arm32(fujimod_emulator_cpu::arm::Register::R(r))).unwrap()
    });
    let (retained, discards) = board.state.borrow().code_shadow[0]
        .as_ref()
        .map_or((0, 0), |shadow| (shadow.retained(), shadow.discards()));
    (board.state(), registers, retained, discards)
}

#[test]
fn retained_translations_run_the_instructions_that_discarded_translations_run() {
    for quantum in [1, 2, 3, 5, 8, 64] {
        // 0x3101 is adds r1,#1; 0x3001 rewrites the instruction unchanged.
        for patch in [0x3101, 0x3001] {
            let (discarded, discarded_registers, _, _) = run(quantum, false, patch);
            let (kept, kept_registers, retained, _) = run(quantum, true, patch);
            assert_eq!(kept_registers, discarded_registers, "quantum {quantum}");
            assert_eq!(kept, discarded, "quantum {quantum}");
            assert!(retained > 0, "quantum {quantum} never kept a translation");
            if patch == 0x3101 {
                assert!(kept_registers[1] > 0, "the peer's patch never ran");
                assert!(kept_registers[0] > 0, "the original loop never ran");
            } else {
                assert_eq!(kept_registers[1], 0);
            }
        }
    }
}

#[test]
fn a_host_write_between_run_windows_reaches_retained_translations() {
    for quantum in [1, 3, 8, 64] {
        // The peer rewrites the instruction unchanged; only the host patches it.
        let (discarded, discarded_registers, _, _) = run_with(
            quantum,
            false,
            0x3001,
            Some(150),
            ExecutionPolicy::Exact,
            false,
        );
        let (kept, kept_registers, retained, _) = run_with(
            quantum,
            true,
            0x3001,
            Some(150),
            ExecutionPolicy::Exact,
            false,
        );
        assert_eq!(kept_registers, discarded_registers, "quantum {quantum}");
        assert_eq!(kept, discarded, "quantum {quantum}");
        assert!(retained > 0, "quantum {quantum} never kept a translation");
        assert!(kept_registers[0] > 0 && kept_registers[1] > 0, "quantum {quantum}");
    }
}

/// The interactive policy runs these two private-engine processors in ordered
/// slices too, so the shadow keeps translations there with the same result.
#[test]
fn the_interactive_policy_keeps_translations_with_the_same_result() {
    let policy = ExecutionPolicy::Interactive;
    for quantum in [1, 3, 8, 64] {
        for (patch, host_write_at) in [(0x3101, None), (0x3001, None), (0x3001, Some(150))] {
            let (discarded, discarded_registers, _, _) =
                run_with(quantum, false, patch, host_write_at, policy, false);
            let (kept, kept_registers, retained, _) =
                run_with(quantum, true, patch, host_write_at, policy, false);
            assert_eq!(kept_registers, discarded_registers, "quantum {quantum} patch {patch:#x}");
            assert_eq!(kept, discarded, "quantum {quantum} patch {patch:#x}");
            assert!(retained > 0, "quantum {quantum} never kept a translation");
            assert!(kept_registers[1] > 0 || patch == 0x3001 && host_write_at.is_none());
        }
    }
}

/// The M4 stores into its loop's page from code in another page, in slices
/// that may not run the loop's page. Its own stores keep that page's copy
/// current under either policy, so they add no discard to the peer's patch.
#[test]
fn own_stores_into_a_code_page_keep_translations_under_either_policy() {
    for policy in [ExecutionPolicy::Exact, ExecutionPolicy::Interactive] {
        for quantum in [1, 3, 8, 64] {
            let at = format!("{policy:?} quantum {quantum}");
            let (_, _, _, patch_discards) = run_with(quantum, true, 0x3101, None, policy, false);
            let (discarded, discarded_registers, _, _) =
                run_with(quantum, false, 0x3101, None, policy, true);
            let (kept, kept_registers, retained, discards) =
                run_with(quantum, true, 0x3101, None, policy, true);
            assert_eq!(kept_registers, discarded_registers, "{at}");
            assert_eq!(kept, discarded, "{at}");
            assert!(retained > 0, "{at}: never kept a translation");
            assert!(kept_registers[0] > 0 && kept_registers[1] > 0, "{at}");
            assert!(discards <= patch_discards, "{at}: {discards} discards");
        }
    }
}
