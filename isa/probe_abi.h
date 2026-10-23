// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// probe_abi.h - layout shared by fw/probe_fw.c and isa_probe.cpp.
//
// The host loads the image into the tile's local GDDR through the cached
// alias and zeroes struct probe_results, which sits RESULTS_OFFSET bytes
// after the image, reached by both sides through the uncached alias. The
// firmware fills it in phase by phase, bumping `progress` before each step so
// that the host can name the step if the hart hangs, and parks in wfi when it
// reaches ST_DONE.
#pragma once

#include <stdint.h>

#define PROBE_MAGIC     0x58494E5350524F42ULL   // "XINSPROB"
#define RESULTS_OFFSET  0x10000ULL              // image + stack must fit below this (fw/link.ld)

enum probe_kind { P_SCALAR, P_C16, P_VECTOR };

enum probe_state {
    ST_BOOT = 1,      // firmware running
    ST_CSRS,          // reading every CSR number
    ST_WARL,          // write/read-back probes of satp, menvcfg, PMP, mcounteren
    ST_PROBES,        // instruction probes, progress = probe index
    ST_SWEEP,         // custom-opcode sweep, progress = encoding index
    ST_DONE,
    ST_TRAPPED = 0xDEAD,
};

// Input flags, written by the host before release.
#define FLAG_SWEEP 1ULL

// vtypes each vector probe is tried at: tail and mask agnostic, SEW 8..64,
// LMUL 1 and 2. Bit i of probe_ok corresponds to VTYPES[i].
#define VTYPE(sew_log2_minus3, lmul_log2) (0xC0u | ((sew_log2_minus3) << 3) | (lmul_log2))
#define NUM_VTYPES 8
#define VTYPE_LIST { VTYPE(0, 0), VTYPE(1, 0), VTYPE(2, 0), VTYPE(3, 0), \
                     VTYPE(0, 1), VTYPE(1, 1), VTYPE(2, 1), VTYPE(3, 1) }
#define VTYPE_SEW(i)  (8u << ((i) & 3))
#define VTYPE_LMUL(i) (1u << ((i) >> 2))

#define MAX_PROBES 128
#define NO_TRAP    0xFFFFFFFFu

// Custom-opcode sweep: R-type encodings with rd = x24/v24 and rs1 = rs2 =
// x16/v16 (a6, pointing into the scratch line), for each custom opcode,
// funct3 and funct7. Vector registers 16 and 24 keep register groups aligned
// up to EMUL 8, so widening and narrowing forms are not rejected for that
// reason alone. Run under e8,m1, e16,m1 and e32,m1.
#define SWEEP_OPCODES   4    // custom-0 0x0B, custom-1 0x2B, custom-2 0x5B, custom-3 0x7B
#define SWEEP_COUNT     (SWEEP_OPCODES * 8 * 128)
#define SWEEP_OPCODE(n) (0x0Bu | ((n) << 5) | ((n) >= 2 ? 0x10u : 0))   // 0x0B 0x2B 0x5B 0x7B
#define SWEEP_WORD(i)   ((((i) & 127u) << 25) | (16u << 20) | (16u << 15) | ((((i) >> 7) & 7u) << 12) | \
                         (24u << 7) | SWEEP_OPCODE((i) >> 10))
#define SWEEP_PASSES    3    // VTYPES[0..2]: e8,m1  e16,m1  e32,m1

struct probe_results {
    // 0x00: control
    uint64_t magic, state, progress, flags;
    uint64_t trap_cause, trap_epc, trap_tval, trap_state;   // unexpected trap: where and in which phase

    // 0x40: identification, read directly
    uint64_t mvendorid, marchid, mimpid, mhartid;
    uint64_t misa, mstatus, mconfigptr, vlenb;

    // 0x80: WARL write/read-back probes. *_ok is 1 when the CSR accepted the
    // accesses at all.
    uint64_t menvcfg_ok, menvcfg_rb;        // after writing all ones
    uint64_t satp_ok, satp_modes;           // bit m set: MODE m was accepted
    uint64_t satp_asid_rb;                  // ASID field after writing all ones (Sv39)
    uint64_t mcounteren_ok, mcounteren_rb;  // after writing all ones
    uint64_t pmp_entries;                   // pmpaddrN that accepted a write
    uint64_t pmpaddr0_rb;                   // pmpaddr0 after all ones, with pmp0cfg = 0
    uint64_t vtype_ok;                      // bit i: vsetvl accepted VTYPES[i]
    uint64_t nprobes;
    uint64_t pad0[5];

    // 0x100: CSR sweep
    uint64_t csr_present[4096 / 64];
    uint64_t csr_value[4096];

    // instruction probes
    uint32_t probe_ok[MAX_PROBES];          // scalar: bit 0; vector: bit per VTYPES entry
    uint32_t probe_cause[MAX_PROBES];       // mcause of the first trap, NO_TRAP if none

    // custom-opcode sweep
    uint64_t sweep_ok[SWEEP_COUNT / 64];    // executed without trapping
    uint64_t sweep_other[SWEEP_COUNT / 64]; // trapped with something other than illegal instruction
};

#ifdef __cplusplus
#define PROBE_STATIC_ASSERT static_assert
#else
#define PROBE_STATIC_ASSERT _Static_assert
#endif
PROBE_STATIC_ASSERT(sizeof(struct probe_results) % 8 == 0, "results must be whole 64-bit words");
PROBE_STATIC_ASSERT(sizeof(struct probe_results) <= 0x10000, "results must fit in 64 KiB");
