// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// dma_fw.c - bare-metal firmware for one Blackhole L2CPU tile that drives the
// tile's DW_ahb_dmac on behalf of a host over a mailbox.
//
// Hart 0 runs this; harts 1..3 park in wfi (see start.S). The image is
// position independent (-mcmodel=medany, no absolute pointers) and is
// loaded by the host into the tile's local GDDR through the *cached* alias
// (0x4000_3000_0000 + offset). The mailbox lives 64 KiB after the image
// start, accessed by both sides through the *uncached* alias so no cache
// maintenance is needed.
//
// On boot the firmware ECC-scrubs the L3 LIM (SiFive manual 18.1.1: stores
// from the processor), then loops waiting for commands.
//
// Mailbox (all 64-bit words, offsets from MBOX):
//   0x00 magic        firmware writes MBOX_MAGIC once running
//   0x08 state        ST_*
//   0x10 cmd          host writes CMD_*; firmware writes 0 when finished
//   0x18 seq          incremented per completed command
//   0x20 status       result of the last command (0 = ok)
//   0x28 cycles       mcycle spent in the last command's DMA (or scrub)
//   0x30 trap_cause   } written by the trap handler, state becomes ST_TRAPPED
//   0x38 trap_epc     }
//   0x40 trap_tval    }
//   0x80 args[16]
//   0x100 res[16]
//
// CMD_DMA args: [0] src_kind [1] src_x [2] src_y [3] src_addr
//               [4] dst_kind [5] dst_x [6] dst_y [7] dst_addr
//               [8] bytes [9] width (1..32) [10] burst items [11] channel
//   kind 0 PHYS      X280 physical address (LIM, local GDDR, ...) via DMA
//                    master 1 and an inbound DMA TLB
//   kind 1 NOC_SMALL (x, y, addr) through a 2 MiB X280 NOC window and DMA
//                    master 0 via an outbound DMA TLB
//   kind 2 NOC_LARGE same through a 128 GiB window (for host IOVAs)
//   kind 3 XBAR      crossbar address (local GDDR offset) via master 0
//   res[0] = blocks, res[1] = RAW_ERR, res[2] = RAW_TFR

#include <stdint.h>

// ---------------------------------------------------------------------------
// Addresses as seen by the X280
// ---------------------------------------------------------------------------

#define CACHED_ALIAS_DELTA   0x400000000000ULL
#define MBOX_OFFSET          0x10000ULL

#define PERIPH_BASE          0x20000000UL       // external peripherals
#define NOCWIN_SMALL_CFG     (PERIPH_BASE + 0x000)
#define NOCWIN_LARGE_CFG     (PERIPH_BASE + 0xE00)
#define L2CPU_REG_BASE       (PERIPH_BASE + 0x10000)
#define OUTBOUND_DMA_TLB0    (L2CPU_REG_BASE + 0x200)
#define INBOUND_DMA_TLB0     (L2CPU_REG_BASE + 0x240)
#define DMAC_BASE            0x20080000UL

#define LIM_BASE             0x08000000UL
#define LIM_END              0x081E0000UL       // 15 ways x 128 KiB
#define LLI_AREA             0x081D0000UL       // last 64 KiB of LIM

#define SYSPORT_BASE         0x30000000UL       // uncached GDDR / crossbar view
#define XBAR_NOC_SMALL       0x000400000000ULL  // crossbar: small windows
#define XBAR_NOC_LARGE       0x080400000000ULL  // crossbar: large windows
#define SMALL_WIN            (2ULL << 20)
#define LARGE_WIN            (128ULL << 30)

// DW_ahb_dmac
#define CH_STRIDE  0x58
#define R_SAR 0x00
#define R_DAR 0x08
#define R_LLP 0x10
#define R_CTL 0x18
#define R_CFG 0x40
#define R_SGR 0x48
#define R_DSR 0x50
#define R_RAW_TFR 0x2C0
#define R_RAW_ERR 0x2E0
#define R_MASK_TFR 0x310
#define R_MASK_BLOCK 0x318
#define R_MASK_SRCTRAN 0x320
#define R_MASK_DSTTRAN 0x328
#define R_MASK_ERR 0x330
#define R_CLEAR_TFR 0x338
#define R_CLEAR_BLOCK 0x340
#define R_CLEAR_SRCTRAN 0x348
#define R_CLEAR_DSTTRAN 0x350
#define R_CLEAR_ERR 0x358
#define R_DMA_CFG 0x398
#define R_CH_EN 0x3A0
#define R_DMA_ID 0x3A8
#define R_COMP_ID 0x3F8
#define MAX_BLOCK_TS 4095u

// ---------------------------------------------------------------------------
// Mailbox
// ---------------------------------------------------------------------------

#define MBOX_MAGIC   0x58323830444D4131ULL   // "X280DMA1"
enum { ST_BOOT = 0, ST_SCRUB = 1, ST_READY = 2, ST_BUSY = 3, ST_PARKED = 4, ST_TRAPPED = 0xDEAD };
enum { CMD_NOP = 1, CMD_DMA = 2, CMD_RD64 = 3, CMD_WR64 = 4, CMD_DUMP = 5, CMD_PARK = 6 };
enum { KIND_PHYS = 0, KIND_NOC_SMALL = 1, KIND_NOC_LARGE = 2, KIND_XBAR = 3 };
enum { E_OK = 0, E_BAD_ARG = 1, E_RANGE = 2, E_TIMEOUT = 3, E_DMA_ERR = 4, E_NO_TFR = 5, E_UNKNOWN_CMD = 6 };

struct mbox {
    volatile uint64_t magic, state, cmd, seq, status, cycles;
    volatile uint64_t trap_cause, trap_epc, trap_tval;
    volatile uint64_t pad[7];
    volatile uint64_t args[16];
    volatile uint64_t res[16];
};

extern char _start[];   // linker/start.S symbol; PC-relative, so it is the real load address

static struct mbox* mbox_ptr(void) {
    uint64_t code = (uint64_t)(uintptr_t)_start;
    return (struct mbox*)(uintptr_t)(code - CACHED_ALIAS_DELTA + MBOX_OFFSET);
}

// ---------------------------------------------------------------------------
// Low-level helpers
// ---------------------------------------------------------------------------

static inline void fence(void) { __asm__ volatile("fence" ::: "memory"); }
static inline uint64_t rd_cycle(void) { uint64_t v; __asm__ volatile("csrr %0, mcycle" : "=r"(v)); return v; }
static inline void wr64(uint64_t a, uint64_t v) { *(volatile uint64_t*)(uintptr_t)a = v; }
static inline uint64_t rd64(uint64_t a) { return *(volatile uint64_t*)(uintptr_t)a; }
static inline void wr32(uint64_t a, uint32_t v) { *(volatile uint32_t*)(uintptr_t)a = v; }
static inline uint32_t rd32(uint64_t a) { return *(volatile uint32_t*)(uintptr_t)a; }

static inline void dmac_w(uint32_t off, uint64_t v) { wr64(DMAC_BASE + off, v); fence(); }
static inline uint64_t dmac_r(uint32_t off) { return rd64(DMAC_BASE + off); }
static inline void dmac_wch(unsigned ch, uint32_t off, uint64_t v) { dmac_w(ch * CH_STRIDE + off, v); }

static inline uint64_t we(unsigned ch, int set) { return (1ULL << (8 + ch)) | ((uint64_t)(set != 0) << ch); }

// ---------------------------------------------------------------------------
// LIM ECC scrub (the sanctioned way: 64-bit stores from the hart)
// ---------------------------------------------------------------------------

static void scrub_lim(void) {
    for (uint64_t a = LIM_BASE; a < LIM_END; a += 8) wr64(a, 0);
    fence();
}

// ---------------------------------------------------------------------------
// The three address layers
// ---------------------------------------------------------------------------

// Layer 3: X280 NOC windows.
// local_offset holds the NOC address in units of the window size: its low 43
// (small) or 27 (large) bits become address bits 63:21 or 63:37.
// Windows that the DMA reads from use strict AXI ordering (ordering = 1):
// DRAM tiles return read responses out of order and the AXI bridge stalls
// on that with the default ordering. Writes use the default.
static uint32_t win_props(uint32_t x, uint32_t y, int strict) { return (x & 0x3F) | ((y & 0x3F) << 6) | ((uint32_t)(strict ? 1 : 0) << 25); }

static uint64_t set_small_window(unsigned n, uint32_t x, uint32_t y, uint64_t noc_addr_aligned, int strict) {
    uint64_t cfg = NOCWIN_SMALL_CFG + 16ULL * n;
    wr64(cfg, noc_addr_aligned >> 21);
    wr32(cfg + 8, win_props(x, y, strict));           // unicast, NOC 0
    wr32(cfg + 12, 0);
    fence();
    return XBAR_NOC_SMALL + SMALL_WIN * n;
}
static uint64_t set_large_window(unsigned n, uint32_t x, uint32_t y, uint64_t noc_addr_aligned, int strict) {
    uint64_t cfg = NOCWIN_LARGE_CFG + 12ULL * n;
    wr32(cfg, (uint32_t)(noc_addr_aligned >> 37));
    wr32(cfg + 4, win_props(x, y, strict));
    wr32(cfg + 8, 0);
    fence();
    return XBAR_NOC_LARGE + LARGE_WIN * n;
}

// Layer 2: DMA address-expansion TLBs.
static void set_outbound_tlb(unsigned slot, uint64_t xbar_base_256m) { wr32(OUTBOUND_DMA_TLB0 + 4 * slot, (uint32_t)(xbar_base_256m >> 28)); fence(); }
static void set_inbound_tlb(unsigned slot, uint64_t phys_base_1g)     { wr32(INBOUND_DMA_TLB0 + 4 * slot, (uint32_t)(phys_base_1g >> 30)); fence(); }

struct ep { unsigned master; uint32_t addr; };

// Turn a (kind, x, y, addr, bytes) endpoint into a DMAC master + 32-bit address.
// Source endpoints use window 0 / outbound slot 2 / inbound slot 1;
// destinations use window 1 / outbound slot 3 / inbound slot 2.
static int resolve(uint64_t kind, uint64_t x, uint64_t y, uint64_t addr, uint64_t bytes, int is_dst, struct ep* out) {
    const unsigned win = is_dst ? 1 : 0, oslot = is_dst ? 3 : 2, islot = is_dst ? 2 : 1;
    uint64_t xbar;
    switch (kind) {
    case KIND_PHYS: {
        if (addr + bytes <= (1ULL << 30)) { out->master = 1; out->addr = (uint32_t)addr; return E_OK; }   // identity slot 0
        uint64_t base = addr & ~((1ULL << 30) - 1);
        if (addr + bytes > base + (1ULL << 30)) return E_RANGE;
        set_inbound_tlb(islot, base);
        out->master = 1; out->addr = (uint32_t)((islot << 30) | (addr - base));
        return E_OK;
    }
    case KIND_NOC_SMALL: {
        uint64_t base = addr & ~(SMALL_WIN - 1);
        if (addr + bytes > base + SMALL_WIN) return E_RANGE;
        xbar = set_small_window(win, (uint32_t)x, (uint32_t)y, base, !is_dst) + (addr - base);
        break;
    }
    case KIND_NOC_LARGE: {
        uint64_t base = addr & ~(LARGE_WIN - 1);
        xbar = set_large_window(win, (uint32_t)x, (uint32_t)y, base, !is_dst) + (addr - base);
        break;
    }
    case KIND_XBAR:
        xbar = addr;
        break;
    default:
        return E_BAD_ARG;
    }
    uint64_t obase = xbar & ~((1ULL << 28) - 1);
    if (xbar + bytes > obase + (1ULL << 28)) return E_RANGE;
    set_outbound_tlb(oslot, obase);
    out->master = 0; out->addr = (uint32_t)((oslot << 28) | (xbar - obase));
    return E_OK;
}

// ---------------------------------------------------------------------------
// DMA transfer
// ---------------------------------------------------------------------------

static int width_code(uint64_t b) { switch (b) { case 1: return 0; case 2: return 1; case 4: return 2; case 8: return 3; case 16: return 4; case 32: return 5; } return -1; }
static int msize_code(uint64_t n) { switch (n) { case 1: return 0; case 4: return 1; case 8: return 2; case 16: return 3; case 32: return 4; case 64: return 5; case 128: return 6; case 256: return 7; } return -1; }

static uint64_t make_ctl(unsigned sms, unsigned dms, int w, int m, uint32_t block_ts, int chain) {
    uint64_t ctl = 1;                               // INT_EN
    ctl |= (uint64_t)w << 1 | (uint64_t)w << 4;     // DST/SRC_TR_WIDTH
    ctl |= (uint64_t)m << 11 | (uint64_t)m << 14;   // DEST/SRC_MSIZE
    ctl |= (uint64_t)dms << 23 | (uint64_t)sms << 25;
    ctl |= (uint64_t)chain << 27 | (uint64_t)chain << 28;
    ctl |= (uint64_t)block_ts << 32;
    return ctl;
}

struct lli { uint32_t sar, dar, llp, ctl_lo, ctl_hi, sstat, dstat, pad; };

static int dma_copy(struct mbox* mb) {
    struct ep src, dst;
    uint64_t bytes = mb->args[8], width = mb->args[9], burst = mb->args[10], ch = mb->args[11];
    int w = width_code(width), m = msize_code(burst);
    if (w < 0 || m < 0 || ch > 1 || bytes == 0 || (bytes % width)) return E_BAD_ARG;
    int e = resolve(mb->args[0], mb->args[1], mb->args[2], mb->args[3], bytes, 0, &src);
    if (e) return e;
    e = resolve(mb->args[4], mb->args[5], mb->args[6], mb->args[7], bytes, 1, &dst);
    if (e) return e;
    if ((src.addr | dst.addr) & (width - 1)) return E_BAD_ARG;

    const uint64_t max_block = (uint64_t)MAX_BLOCK_TS * width;
    const uint32_t nblocks = (uint32_t)((bytes + max_block - 1) / max_block);
    if (nblocks > 2048) return E_RANGE;
    mb->res[0] = nblocks;

    dmac_w(R_DMA_CFG, 1);
    if ((dmac_r(R_CH_EN) >> ch) & 1) dmac_w(R_CH_EN, we(ch, 0));
    uint64_t bit = 1ULL << ch;
    dmac_w(R_CLEAR_TFR, bit); dmac_w(R_CLEAR_BLOCK, bit); dmac_w(R_CLEAR_SRCTRAN, bit); dmac_w(R_CLEAR_DSTTRAN, bit); dmac_w(R_CLEAR_ERR, bit);
    dmac_w(R_MASK_TFR, we(ch, 1)); dmac_w(R_MASK_BLOCK, we(ch, 1)); dmac_w(R_MASK_ERR, we(ch, 1));
    dmac_w(R_MASK_SRCTRAN, we(ch, 0)); dmac_w(R_MASK_DSTTRAN, we(ch, 0));
    dmac_wch(ch, R_CFG, (uint64_t)ch << 5 | 1ULL << 10 | 1ULL << 11);   // priority, no hardware handshake
    dmac_wch(ch, R_SGR, 0);
    dmac_wch(ch, R_DSR, 0);
    dmac_wch(ch, R_SAR, src.addr);
    dmac_wch(ch, R_DAR, dst.addr);
    if (nblocks == 1) {
        dmac_wch(ch, R_LLP, 0);
        dmac_wch(ch, R_CTL, make_ctl(src.master, dst.master, w, m, (uint32_t)(bytes / width), 0));
    } else {
        // Linked list in the (scrubbed) LIM, fetched through master 1.
        struct lli* list = (struct lli*)(uintptr_t)(LLI_AREA + 0x8000 * ch);
        uint64_t off = 0;
        for (uint32_t i = 0; i < nblocks; i++) {
            uint64_t len = bytes - off < max_block ? bytes - off : max_block;
            int last = (i + 1 == nblocks);
            uint64_t ctl = make_ctl(src.master, dst.master, w, m, (uint32_t)(len / width), !last);
            list[i].sar = (uint32_t)(src.addr + off);
            list[i].dar = (uint32_t)(dst.addr + off);
            list[i].llp = last ? 0 : (uint32_t)((uintptr_t)&list[i + 1]) | 1;   // LMS = master 1
            list[i].ctl_lo = (uint32_t)ctl;
            list[i].ctl_hi = (uint32_t)(ctl >> 32);
            list[i].sstat = list[i].dstat = list[i].pad = 0;
            off += len;
        }
        fence();
        dmac_wch(ch, R_LLP, (uint32_t)(uintptr_t)list | 1);
        dmac_wch(ch, R_CTL, make_ctl(src.master, dst.master, w, m, 0, 1));
    }

    uint64_t t0 = rd_cycle();
    dmac_w(R_CH_EN, we(ch, 1));
    int status = E_OK;
    while ((dmac_r(R_CH_EN) >> ch) & 1) {
        if (rd_cycle() - t0 > 3500000000ULL) { status = E_TIMEOUT; break; }    // ~2 s at 1.75 GHz
    }
    mb->cycles = rd_cycle() - t0;
    uint64_t raw_err = dmac_r(R_RAW_ERR), raw_tfr = dmac_r(R_RAW_TFR);
    mb->res[1] = raw_err;
    mb->res[2] = raw_tfr;
    if (status == E_TIMEOUT) {
        dmac_w(R_CH_EN, we(ch, 0));
    } else if ((raw_err >> ch) & 1) {
        status = E_DMA_ERR;
    } else if (!((raw_tfr >> ch) & 1)) {
        status = E_NO_TFR;
    }
    dmac_w(R_CLEAR_TFR, bit); dmac_w(R_CLEAR_BLOCK, bit); dmac_w(R_CLEAR_ERR, bit);
    return status;
}

static void dump_dmac(struct mbox* mb) {
    static const uint32_t regs[16] = { R_COMP_ID, R_DMA_ID, R_DMA_CFG, R_CH_EN, R_RAW_TFR, R_RAW_ERR,
                                       R_SAR, R_DAR, R_LLP, R_CTL, R_CFG,
                                       CH_STRIDE + R_SAR, CH_STRIDE + R_DAR, CH_STRIDE + R_LLP, CH_STRIDE + R_CTL, CH_STRIDE + R_CFG };
    for (int i = 0; i < 16; i++) mb->res[i] = dmac_r(regs[i]);
}

// ---------------------------------------------------------------------------
// Entry points (called from start.S)
// ---------------------------------------------------------------------------

void trap_handler_c(uint64_t cause, uint64_t epc, uint64_t tval) {
    struct mbox* mb = mbox_ptr();
    mb->trap_cause = cause;
    mb->trap_epc = epc;
    mb->trap_tval = tval;
    mb->state = ST_TRAPPED;
    fence();
    for (;;) __asm__ volatile("wfi");
}

void main_hart0(void) {
    struct mbox* mb = mbox_ptr();
    mb->state = ST_SCRUB;
    mb->magic = MBOX_MAGIC;
    fence();

    uint64_t t0 = rd_cycle();
    scrub_lim();
    mb->cycles = rd_cycle() - t0;
    mb->state = ST_READY;
    fence();

    for (;;) {
        uint64_t cmd = mb->cmd;
        if (cmd == 0) continue;
        mb->state = ST_BUSY;
        int status;
        switch (cmd) {
        case CMD_NOP:  status = E_OK; break;
        case CMD_DMA:  status = dma_copy(mb); break;
        case CMD_RD64: mb->res[0] = rd64(mb->args[0]); status = E_OK; break;
        case CMD_WR64: wr64(mb->args[0], mb->args[1]); fence(); status = E_OK; break;
        case CMD_DUMP: dump_dmac(mb); status = E_OK; break;
        case CMD_PARK:
            mb->status = E_OK; mb->seq = mb->seq + 1; mb->state = ST_PARKED; mb->cmd = 0; fence();
            for (;;) __asm__ volatile("wfi");
        default:       status = E_UNKNOWN_CMD; break;
        }
        mb->status = status;
        mb->seq = mb->seq + 1;
        mb->state = ST_READY;
        fence();
        mb->cmd = 0;      // ack last, after results are visible
        fence();
    }
}
