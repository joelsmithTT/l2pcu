// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// probe_fw.c - bare-metal firmware that finds out which ISA extensions an
// X280 hart implements, by trying them.
//
// Hart 0 runs this in M-mode; harts 1..3 park in wfi (see start.S). The
// image is position independent and loaded by the host into local GDDR
// through the cached alias; results go to struct probe_results
// (../probe_abi.h), RESULTS_OFFSET after the image, through the uncached
// alias.
//
// Every probe is executed from a small code buffer: the instruction (or
// instructions) under test followed by `ret`. The firmware writes the buffer,
// runs fence.i and calls it with a0 = a 64-byte-aligned scratch line, a1 and a2
// as arguments. start.S's trap handler counts traps while a probe is armed and
// skips the faulting instruction, so "did it trap, and with which mcause" is
// the whole measurement. The firmware is built for rv64imac, so the compiler
// never touches F or V state and the probes can clobber both freely.

#include <stdint.h>

#include "../probe_abi.h"
#include "../probes.h"

#define CACHED_ALIAS_DELTA 0x400000000000ULL

#define CSR_MSTATUS  0x300
#define CSR_MISA     0x301
#define CSR_MENVCFG  0x30A
#define CSR_MCOUNTEREN 0x306
#define CSR_PMPCFG0  0x3A0
#define CSR_PMPADDR0 0x3B0
#define CSR_SATP     0x180
#define CSR_FCSR     0x003
#define CSR_VTYPE    0xC21
#define CSR_VLENB    0xC22
#define CSR_MVENDORID 0xF11
#define CSR_MARCHID  0xF12
#define CSR_MIMPID   0xF13
#define CSR_MHARTID  0xF14
#define CSR_MCONFIGPTR 0xF15

#define MSTATUS_FS_DIRTY (3ULL << 13)
#define MSTATUS_VS_DIRTY (3ULL << 9)
#define CAUSE_ILLEGAL_INSN 2

// Shared with start.S; the offsets there must match.
struct trap_save { uint64_t t1, t2, armed, cause, count; };
volatile struct trap_save trap_save;

extern char _start[];   // PC-relative, so this is the real load address

static volatile struct probe_results* results(void) {
    return (volatile struct probe_results*)(uintptr_t)((uint64_t)(uintptr_t)_start - CACHED_ALIAS_DELTA + RESULTS_OFFSET);
}

static inline void fence(void) { __asm__ volatile("fence" ::: "memory"); }

void trap_handler_c(uint64_t cause, uint64_t epc, uint64_t tval) {
    volatile struct probe_results* r = results();
    r->trap_cause = cause;
    r->trap_epc = epc;
    r->trap_tval = tval;
    r->trap_state = r->state;
    fence();
    r->state = ST_TRAPPED;
    fence();
    for (;;) __asm__ volatile("wfi");
}

// ---------------------------------------------------------------------------
// The code buffer
// ---------------------------------------------------------------------------

static uint32_t code[8] __attribute__((aligned(64)));
static uint64_t scratch[32] __attribute__((aligned(256)));   // its own cache lines, for cbo.* and AMOs

struct outcome { uint64_t traps, cause, value; };

// Runs code[] with a0 = scratch, a1, a2, and a6 = a1 (for the opcode
// sweep). Code that wants to return a value stores it to 0(a0), which comes
// back as .value. s8 is clobbered too: the sweep writes x24.
static struct outcome run(uint64_t a1v, uint64_t a2v) {
    scratch[0] = 0;
    fence();
    __asm__ volatile("fence.i" ::: "memory");
    trap_save.count = 0;
    trap_save.cause = 0;
    trap_save.armed = 1;
    register uint64_t a0 __asm__("a0") = (uint64_t)(uintptr_t)scratch;
    register uint64_t a1 __asm__("a1") = a1v;
    register uint64_t a2 __asm__("a2") = a2v;
    register uint64_t a6 __asm__("a6") = a1v;
    __asm__ volatile("jalr ra, 0(%4)"
                     : "+r"(a0), "+r"(a1), "+r"(a2), "+r"(a6)
                     : "r"(code)
                     : "ra", "t0", "t1", "t2", "t3", "t4", "t5", "t6", "a3", "a4", "a5", "a7", "s8", "memory");
    trap_save.armed = 0;
    struct outcome o = { trap_save.count, trap_save.cause, scratch[0] };
    return o;
}

static struct outcome run1(uint32_t insn, uint64_t a1v, uint64_t a2v) {
    code[0] = insn;
    code[1] = INSN_RET;
    return run(a1v, a2v);
}

// A 16-bit instruction, then c.nop to realign, then ret.
static struct outcome run_c16(uint16_t insn) {
    volatile uint16_t* h = (volatile uint16_t*)code;
    h[0] = insn;
    h[1] = 0x0001;
    code[1] = INSN_RET;
    return run(0, 0);
}

static int csr_read(unsigned csr, uint64_t* v) {
    code[0] = INSN_CSRRS_T3_X0 | (csr << 20);
    code[1] = INSN_SD_T3_A0;
    code[2] = INSN_RET;
    struct outcome o = run(0, 0);
    *v = o.value;
    return o.traps == 0;
}

// Write `v`, read the CSR back, restore the old value. Returns 1 and the
// read-back value if all three accesses worked.
static int csr_write_probe(unsigned csr, uint64_t v, uint64_t* rb) {
    code[0] = INSN_CSRRW_T3_A1 | (csr << 20);
    code[1] = INSN_CSRRS_T4_X0 | (csr << 20);
    code[2] = INSN_CSRRW_X0_T3 | (csr << 20);
    code[3] = INSN_SD_T4_A0;
    code[4] = INSN_RET;
    struct outcome o = run(v, 0);
    *rb = o.value;
    return o.traps == 0;
}

// ---------------------------------------------------------------------------
// Phases
// ---------------------------------------------------------------------------

static void read_csrs(volatile struct probe_results* r) {
    r->state = ST_CSRS;
    for (unsigned csr = 0; csr < 4096; csr++) {
        r->progress = csr;
        uint64_t v;
        if (csr_read(csr, &v)) {
            r->csr_present[csr / 64] |= 1ULL << (csr % 64);
            r->csr_value[csr] = v;
        }
    }
    uint64_t v;
    if (csr_read(CSR_MVENDORID, &v)) r->mvendorid = v;
    if (csr_read(CSR_MARCHID, &v)) r->marchid = v;
    if (csr_read(CSR_MIMPID, &v)) r->mimpid = v;
    if (csr_read(CSR_MHARTID, &v)) r->mhartid = v;
    if (csr_read(CSR_MISA, &v)) r->misa = v;
    if (csr_read(CSR_MCONFIGPTR, &v)) r->mconfigptr = v;
    if (csr_read(CSR_VLENB, &v)) r->vlenb = v;
}

static void warl_probes(volatile struct probe_results* r) {
    r->state = ST_WARL;
    uint64_t v;
    r->progress = 0;
    r->menvcfg_ok = csr_write_probe(CSR_MENVCFG, ~0ULL, &v);
    r->menvcfg_rb = v;

    // satp: a write with an unsupported MODE has no effect at all. M-mode
    // ignores satp (MPRV is clear), so this changes nothing for us.
    r->progress = 1;
    r->satp_ok = 1;
    for (unsigned mode = 1; mode < 16; mode++) {
        if (!csr_write_probe(CSR_SATP, (uint64_t)mode << 60, &v)) { r->satp_ok = 0; break; }
        if (v >> 60 == mode) r->satp_modes |= 1ULL << mode;
    }
    if (r->satp_ok && (r->satp_modes & (1u << 8)) && csr_write_probe(CSR_SATP, (8ULL << 60) | (0xFFFFULL << 44), &v))
        r->satp_asid_rb = (v >> 44) & 0xFFFF;

    r->progress = 2;
    r->mcounteren_ok = csr_write_probe(CSR_MCOUNTEREN, 0xFFFFFFFFULL, &v);
    r->mcounteren_rb = v;

    // PMP: only touch entries whose config byte is zero (off, unlocked).
    // With pmp0cfg = 0, writing all ones to pmpaddr0 reveals the granularity.
    r->progress = 3;
    for (unsigned i = 0; i < 64; i++) {
        uint64_t cfg;
        if (!csr_read(CSR_PMPCFG0 + (i / 8) * 2, &cfg)) break;
        if ((cfg >> ((i % 8) * 8)) & 0xFF) { r->pmp_entries++; continue; }   // in use, so implemented
        if (csr_write_probe(CSR_PMPADDR0 + i, ~0ULL, &v) && v) {
            r->pmp_entries++;
            if (i == 0) r->pmpaddr0_rb = v;
        }
    }
}

static const uint8_t vtypes[NUM_VTYPES] = VTYPE_LIST;

// vsetvl t3, a1 = ~0 (VLMAX), a2 = vtype; then read vtype back. Returns 1 if
// the vtype is legal (vill clear).
static int set_vtype(unsigned i) {
    if (run1(INSN_VSETVL, ~0ULL, vtypes[i]).traps) return 0;
    uint64_t vt;
    return csr_read(CSR_VTYPE, &vt) && !(vt >> 63);
}

struct probe { uint32_t insn; uint8_t kind; };
#define PROBE_ENTRY(ext, text, insn, kind, sew) { insn, kind },
static const struct probe probes[] = { PROBE_LIST(PROBE_ENTRY) };
#define NPROBES (sizeof(probes) / sizeof(probes[0]))
_Static_assert(NPROBES <= MAX_PROBES, "too many probes");

static void instruction_probes(volatile struct probe_results* r) {
    r->state = ST_PROBES;
    r->nprobes = NPROBES;
    for (unsigned i = 0; i < NUM_VTYPES; i++)
        if (set_vtype(i)) r->vtype_ok |= 1ULL << i;
    for (unsigned p = 0; p < NPROBES; p++) {
        r->progress = p;
        fence();
        uint32_t ok = 0, cause = NO_TRAP;
        if (probes[p].kind == P_VECTOR) {
            for (unsigned i = 0; i < NUM_VTYPES; i++) {
                if (!((r->vtype_ok >> i) & 1) || !set_vtype(i)) continue;
                struct outcome o = run1(probes[p].insn, 0, 0);
                if (!o.traps) ok |= 1u << i;
                else if (cause == NO_TRAP) cause = (uint32_t)o.cause;
            }
            if (!r->vtype_ok) cause = CAUSE_ILLEGAL_INSN;   // no vector unit at all
        } else {
            struct outcome o = probes[p].kind == P_C16 ? run_c16((uint16_t)probes[p].insn) : run1(probes[p].insn, 0, 0);
            if (!o.traps) ok = 1;
            else cause = (uint32_t)o.cause;
        }
        r->probe_ok[p] = ok;
        r->probe_cause[p] = cause;
    }
}

// Every R-type encoding in the four custom opcode spaces. Opt-in: an unknown
// instruction could hang rather than trap.
static void opcode_sweep(volatile struct probe_results* r) {
    r->state = ST_SWEEP;
    for (unsigned pass = 0; pass < SWEEP_PASSES; pass++) {
        int have_v = set_vtype(pass);
        if (pass && !have_v) break;
        for (unsigned i = 0; i < SWEEP_COUNT; i++) {
            r->progress = pass * SWEEP_COUNT + i;
            fence();
            if (have_v) set_vtype(pass);   // a previous encoding may have changed it
            uint64_t s = (uint64_t)(uintptr_t)scratch;
            struct outcome o = run1(SWEEP_WORD(i), s + 64, 0);
            if (!o.traps) r->sweep_ok[i / 64] |= 1ULL << (i % 64);
            else if (o.cause != CAUSE_ILLEGAL_INSN) r->sweep_other[i / 64] |= 1ULL << (i % 64);
        }
    }
}

void main_hart0(void) {
    volatile struct probe_results* r = results();
    r->state = ST_BOOT;
    r->magic = PROBE_MAGIC;
    fence();

    // Turn on the FP and vector units if they exist (the bits are WARL) and
    // put fcsr in a known state, so FP and vector probes fail only for lack
    // of the extension.
    __asm__ volatile("csrs mstatus, %0" :: "r"(MSTATUS_FS_DIRTY | MSTATUS_VS_DIRTY));
    uint64_t v;
    if (csr_read(CSR_MSTATUS, &v)) r->mstatus = v;
    code[0] = 0x00001073u | (CSR_FCSR << 20);   // csrrw zero, fcsr, zero
    code[1] = INSN_RET;
    run(0, 0);

    read_csrs(r);
    warl_probes(r);
    instruction_probes(r);
    if (r->flags & FLAG_SWEEP) opcode_sweep(r);

    fence();
    r->state = ST_DONE;
    fence();
    for (;;) __asm__ volatile("wfi");
}
