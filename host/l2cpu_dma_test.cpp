// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// l2cpu_dma_test - exercise the L2CPU tile DMA engine from the host.
//
//   l2cpu_dma_test probe                 read-only register dump, all L2CPUs
//   l2cpu_dma_test test  [l2cpu]         functional tests (default: all 4)
//   l2cpu_dma_test perf  [l2cpu]         throughput sweep
//
// Nothing here requires the X280 harts to be running.

#include "l2cpu_dma.hpp"

#include <cinttypes>
#include <memory>
#include <random>

using namespace l2dma;

namespace {

// -----------------------------------------------------------------------------
// probe
// -----------------------------------------------------------------------------

void decode_comp_params(Dmac& d) {
    // Component parameter words (DW_ahb_dmac "DMA_COMP_PARAMS_x"), 32-bit each
    // from 0x3C8 to 0x3F4. Decoded loosely per the DW_ahb_dmac databook.
    printf("    comp params:");
    for (uint32_t off = dmac::COMP_PARAMS_BASE; off < dmac::COMP_ID; off += 8) {
        uint64_t v = d.rd(off);
        printf(" [%03x]=%08x,%08x", off, (uint32_t)v, (uint32_t)(v >> 32));
    }
    printf("\n");
    uint64_t p1 = d.rd(0x3F0);            // DMA_COMP_PARAMS_1: [31:0] = DW_PARAMS (0x3F0), [63:32] = ...
    uint32_t dw_params = (uint32_t)p1;
    printf("    DW_PARAMS 0x%08x: nr_chan=%u nr_master=%u data_width[m1..m4]=%u,%u,%u,%u (0=32b 1=64b 2=128b 3=256b)\n",
           dw_params, ((dw_params >> 8) & 7) + 1, ((dw_params >> 11) & 3) + 1,
           (dw_params >> 15) & 3, (dw_params >> 17) & 3, (dw_params >> 19) & 3, (dw_params >> 21) & 3);
    uint64_t p2 = d.rd(0x3E8);            // [31:0] MULTI_BLK_TYPE, [63:32] MAX_BLK_SIZE
    printf("    MULTI_BLK_TYPE 0x%08x  MAX_BLK_SIZE 0x%08x\n", (uint32_t)p2, (uint32_t)(p2 >> 32));
}

int cmd_probe(Device& dev) {
    Noc noc(dev);
    for (int i = 0; i < 4; i++) {
        Dmac d(noc, i);
        d.dump();
        decode_comp_params(d);
        // Same register via two 32-bit reads, to see whether 32-bit accesses
        // are honoured by the 64-bit register file.
        uint32_t lo = noc.read32(d.tile().x, d.tile().y, DMAC_BASE + dmac::COMP_ID);
        uint32_t hi = noc.read32(d.tile().x, d.tile().y, DMAC_BASE + dmac::COMP_ID + 4);
        printf("    COMP_ID via 2x32-bit: %08x %08x\n", hi, lo);
        uint32_t mgmt = d.rd_l2reg(l2reg::X280_MANAGEMENT);
        printf("    X280 mgmt: cease %x halt %x wfi %x debug %x suppress %x\n",
               mgmt & 0xF, (mgmt >> 4) & 0xF, (mgmt >> 8) & 0xF, (mgmt >> 12) & 0xF, (mgmt >> 16) & 0xF);
    }
    return 0;
}

// -----------------------------------------------------------------------------
// test helpers
// -----------------------------------------------------------------------------

struct Pattern {
    std::vector<uint8_t> bytes;
    explicit Pattern(size_t n, uint64_t seed) : bytes(n) {
        std::mt19937_64 rng(seed);
        for (size_t i = 0; i < n; i += 8) {
            uint64_t v = rng();
            memcpy(&bytes[i], &v, std::min<size_t>(8, n - i));
        }
    }
};

int g_pass = 0, g_fail = 0;

void report(const char* name, bool ok, const XferResult& r, const char* detail = "") {
    printf("  [%s] %-44s %s", ok ? "PASS" : "FAIL", name, detail);
    if (r.bytes) printf("  %.3f ms  %.1f MiB/s", r.seconds * 1e3, r.mib_per_s());
    if (r.timed_out) printf("  TIMEOUT");
    if (r.error_irq) printf("  ERR-IRQ");
    printf("\n");
    (ok ? g_pass : g_fail)++;
}

// Copy and verify through host-visible X280 physical addresses.
bool run_copy_phys(Dmac& d, const char* name, unsigned ch, DmaAddr src, DmaAddr dst, uint64_t src_phys, uint64_t dst_phys,
                   size_t bytes, const XferOptions& o, uint64_t seed) {
    Pattern p(bytes, seed);
    d.phys_write(src_phys, p.bytes.data(), bytes);
    d.phys_fill64(dst_phys, 0xDEADBEEFDEADBEEFULL, bytes);
    XferResult r = d.copy(ch, src, dst, bytes, o);
    std::vector<uint8_t> back(bytes);
    d.phys_read(dst_phys, back.data(), bytes);
    size_t first_bad = bytes;
    for (size_t i = 0; i < bytes; i++) if (back[i] != p.bytes[i]) { first_bad = i; break; }
    bool ok = r.ok && first_bad == bytes;
    char detail[96] = "";
    if (first_bad != bytes) snprintf(detail, sizeof(detail), "mismatch @%zu", first_bad);
    report(name, ok, r, detail);
    if (!ok) d.dump();
    return ok;
}

// Copy and verify where source/destination are arbitrary NOC locations.
struct NocLoc { uint16_t x, y; uint64_t addr; };
bool run_copy_noc(Noc& noc, Dmac& d, const char* name, unsigned ch, DmaAddr src, DmaAddr dst, NocLoc s, NocLoc t,
                  size_t bytes, const XferOptions& o, uint64_t seed) {
    Pattern p(bytes, seed);
    noc.write(s.x, s.y, s.addr, p.bytes.data(), bytes);
    noc.fill64(t.x, t.y, t.addr, 0xDEADBEEFDEADBEEFULL, bytes);
    XferResult r = d.copy(ch, src, dst, bytes, o);
    std::vector<uint8_t> back(bytes);
    noc.read(t.x, t.y, t.addr, back.data(), bytes);
    size_t first_bad = bytes;
    for (size_t i = 0; i < bytes; i++) if (back[i] != p.bytes[i]) { first_bad = i; break; }
    bool ok = r.ok && first_bad == bytes;
    char detail[96] = "";
    if (first_bad != bytes) snprintf(detail, sizeof(detail), "mismatch @%zu", first_bad);
    report(name, ok, r, detail);
    if (!ok) d.dump();
    return ok;
}

// -----------------------------------------------------------------------------
// test
// -----------------------------------------------------------------------------

// A Tensix tile to use as a NOC target (NOC0 coordinates). (1,2) is the top-left Tensix on Blackhole.
uint16_t tensix_x = 1, tensix_y = 2;

int cmd_test(Device& dev, int only) {
    Noc noc(dev);
    for (int i = 0; i < 4; i++) {
        if (only >= 0 && i != only) continue;
        Dmac d(noc, i);
        printf("L2CPU %d (%u,%u)\n", i, d.tile().x, d.tile().y);
        d.reset_tlbs_to_identity();
        const bool have_gddr = gddr_present(noc, d.tile());
        if (!have_gddr) printf("  local GDDR does not answer (harvested DRAM tile?) - skipping GDDR cases\n");

        // Scratch areas in L3 LIM (X280 phys), away from anything x280_tester uses.
        const uint64_t limA = X280_LIM_BASE + 0x10'0000;   // 1 MiB in
        const uint64_t limB = X280_LIM_BASE + 0x14'0000;
        // Scratch areas in GDDR, 1 GiB in.
        const uint64_t gddrA = 0x4000'0000ULL;             // offset within GDDR
        const uint64_t gddrB = 0x4000'0000ULL + 0x100'0000;

        XferOptions o;
        o.verbose = false;

        // 1. LIM -> LIM through the L2 master, identity TLB 0. Small.
        run_copy_phys(d, "LIM->LIM  L2/L2   4 KiB  w32", 0,
                      { Master::L2, (uint32_t)limA }, { Master::L2, (uint32_t)limB }, limA, limB, 4096, o, 1);

        // 2. Same with narrower widths.
        for (unsigned w : {1u, 4u, 8u, 16u}) {
            XferOptions ow = o; ow.width_bytes = w; ow.burst_items = 32;
            char n[64]; snprintf(n, sizeof(n), "LIM->LIM  L2/L2   4 KiB  w%u", w);
            run_copy_phys(d, n, 0, { Master::L2, (uint32_t)limA }, { Master::L2, (uint32_t)limB }, limA, limB, 4096, ow, 2 + w);
        }

        // 3. A full single block (4095 words x 32 B) and a multi-block list.
        run_copy_phys(d, "LIM->LIM  L2/L2   130976 B (1 blk)", 0,
                      { Master::L2, (uint32_t)limA }, { Master::L2, (uint32_t)limB }, limA, limB, 4095 * 32, o, 7);
        run_copy_phys(d, "LIM->LIM  L2/L2   256 KiB (LLI)", 0,
                      { Master::L2, (uint32_t)limA }, { Master::L2, (uint32_t)limB }, limA, limB, 256 * 1024, o, 8);

        if (!have_gddr) { d.reset_tlbs_to_identity(); printf("\n"); continue; }

        // 4. GDDR via the L2 master (X280 phys 0x3000_0000 + off, identity TLB 0 covers up to 1 GiB-0x3000_0000;
        //    use inbound slot 1 -> 1 GiB so we can reach GDDR offset 1 GiB).
        d.set_inbound_tlb(1, 1ULL << 30);
        uint64_t gA_phys = X280_GDDR_UNCACHED + gddrA, gB_phys = X280_GDDR_UNCACHED + gddrB;
        run_copy_phys(d, "LIM->GDDR L2/L2   64 KiB", 0,
                      { Master::L2, (uint32_t)limA }, d.l2_addr(1, gA_phys), limA, gA_phys, 64 * 1024, o, 9);
        run_copy_phys(d, "GDDR->LIM L2/L2   64 KiB", 0,
                      d.l2_addr(1, gA_phys), { Master::L2, (uint32_t)limB }, gA_phys, limB, 64 * 1024, o, 10);

        // 5. GDDR via the EXTERN master. Outbound slot 1 -> crossbar 1 GiB (GDDR offset 1 GiB).
        d.set_outbound_tlb(1, XBAR_GDDR_BASE + (1ULL << 30));
        DmaAddr eA = d.extern_addr(1, XBAR_GDDR_BASE + gddrA);
        DmaAddr eB = d.extern_addr(1, XBAR_GDDR_BASE + gddrB);
        run_copy_phys(d, "LIM->GDDR L2/EXT  64 KiB", 0, { Master::L2, (uint32_t)limA }, eA, limA, gA_phys, 64 * 1024, o, 11);
        run_copy_phys(d, "GDDR->LIM EXT/L2  64 KiB", 0, eA, { Master::L2, (uint32_t)limB }, gA_phys, limB, 64 * 1024, o, 12);
        run_copy_phys(d, "GDDR->GDDR EXT/EXT 1 MiB (LLI)", 0, eA, eB, gA_phys, gB_phys, 1 << 20, o, 13);
        run_copy_phys(d, "GDDR->GDDR EXT/EXT 1 MiB ch1", 1, eB, eA, gB_phys, gA_phys, 1 << 20, o, 14);

        // 6. Through an X280 NOC window to a Tensix tile's L1 (and back).
        //    Window 0 -> Tensix (x,y) L1 base; outbound DMA slot 2 covers the
        //    small-window region; master-0 address = slot 2 + window offset.
        {
            const Tile tensix = { tensix_x, tensix_y };
            const uint64_t l1_off = 0x10'0000;   // 1 MiB into the 1.5 MiB L1
            d.set_noc_small_window(0, tensix.x, tensix.y, 0);
            DmaAddr l1 = d.extern_noc_small(2, 0, l1_off);
            NocLoc l1_loc = { tensix.x, tensix.y, l1_off };
            NocLoc gA_loc = { d.tile().x, d.tile().y, gA_phys };
            char n1[64], n2[64];
            snprintf(n1, sizeof(n1), "GDDR->Tensix(%u,%u)L1 EXT/EXT 64 KiB", tensix.x, tensix.y);
            snprintf(n2, sizeof(n2), "Tensix(%u,%u)L1->GDDR EXT/EXT 64 KiB", tensix.x, tensix.y);
            run_copy_noc(noc, d, n1, 0, eA, l1, gA_loc, l1_loc, 64 * 1024, o, 15);
            run_copy_noc(noc, d, n2, 0, l1, eA, l1_loc, gA_loc, 64 * 1024, o, 16);
        }

        d.reset_tlbs_to_identity();
        printf("\n");
    }
    printf("%d passed, %d failed\n", g_pass, g_fail);
    return g_fail ? 1 : 0;
}

// -----------------------------------------------------------------------------
// perf
// -----------------------------------------------------------------------------

void print_pll(Noc& noc) {
    uint32_t c1 = noc.read32(pll::ARC.x, pll::ARC.y, pll::CNTL_1), c5 = noc.read32(pll::ARC.x, pll::ARC.y, pll::CNTL_5);
    printf("L2CPU PLL: fbdiv %u postdiv %u (approx %u MHz)\n", c1 >> 16, c5 & 0xFF, (c1 >> 16) * 25 / 2 / std::max<unsigned>(1, c5 & 0xFF));
}

bool apply_pll(Noc& noc, const char* which) {
    std::string w = which ? which : "";
    if (w == "200") set_l2cpu_pll(noc, pll::MHZ_200);
    else if (w == "800") set_l2cpu_pll(noc, pll::MHZ_800);
    else if (w == "1750") set_l2cpu_pll(noc, pll::MHZ_1750);
    else if (!w.empty()) { fprintf(stderr, "pll: 200 | 800 | 1750\n"); return false; }
    return true;
}

// pll [mhz] [hold_seconds]: set and, while keeping the device open, watch
// whether the ARC firmware puts it back.
int cmd_pll(Device& dev, const char* which, double hold_s) {
    Noc noc(dev);
    if (!apply_pll(noc, which)) return 2;
    print_pll(noc);
    auto t0 = std::chrono::steady_clock::now();
    uint32_t last = noc.read32(pll::ARC.x, pll::ARC.y, pll::CNTL_1) >> 16;
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < hold_s) {
        uint32_t now = noc.read32(pll::ARC.x, pll::ARC.y, pll::CNTL_1) >> 16;
        if (now != last) {
            printf("  fbdiv changed %u -> %u after %.2f s (device still open)\n", last, now,
                   std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
            last = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    if (hold_s > 0) print_pll(noc);
    return 0;
}

int cmd_perf(Device& dev, int only, const char* mhz) {
    Noc noc(dev);
    if (!apply_pll(noc, mhz)) return 2;
    print_pll(noc);
    for (int i = 0; i < 4; i++) {
        if (only >= 0 && i != only) continue;
        Dmac d(noc, i);
        printf("L2CPU %d (%u,%u)\n", i, d.tile().x, d.tile().y);
        d.reset_tlbs_to_identity();
        if (!gddr_present(noc, d.tile())) { printf("  local GDDR does not answer - skipping\n"); continue; }
        d.set_outbound_tlb(1, XBAR_GDDR_BASE + (1ULL << 30));
        d.set_inbound_tlb(1, 1ULL << 30);
        const uint64_t gA = 0x4000'0000ULL, gB = 0x4000'0000ULL + 0x800'0000;
        struct Case { const char* name; DmaAddr src, dst; } cases[] = {
            { "GDDR->GDDR EXT/EXT", d.extern_addr(1, gA), d.extern_addr(1, gB) },
            { "GDDR->GDDR L2/L2  ", d.l2_addr(1, X280_GDDR_UNCACHED + gA), d.l2_addr(1, X280_GDDR_UNCACHED + gB) },
            { "GDDR->GDDR EXT/L2 ", d.extern_addr(1, gA), d.l2_addr(1, X280_GDDR_UNCACHED + gB) },
            { "GDDR->GDDR L2/EXT ", d.l2_addr(1, X280_GDDR_UNCACHED + gA), d.extern_addr(1, gB) },
        };
        for (auto& c : cases) {
            for (uint64_t sz : { 64ull << 10, 1ull << 20, 16ull << 20 }) {
                for (unsigned burst : { 256u }) {
                    XferOptions o; o.width_bytes = 32; o.burst_items = burst; o.timeout_s = 10;
                    XferResult r = d.copy(0, c.src, c.dst, sz, o);
                    printf("  %s %6llu KiB burst %3u : %s %8.3f ms  %8.1f MiB/s\n", c.name, (unsigned long long)(sz >> 10), burst,
                           r.ok ? "ok  " : "FAIL", r.seconds * 1e3, r.mib_per_s());
                    if (!r.ok) { d.dump(); break; }
                }
            }
        }
        // Both channels at once, each moving 16 MiB GDDR->GDDR on the EXT master.
        {
            XferOptions o0, o1; o0.timeout_s = o1.timeout_s = 10;
            o1.lli_dma_addr += 0x8000; o1.lli_x280_phys += 0x8000;   // separate list area for ch1
            const uint64_t sz = 16ull << 20;
            DmaAddr s0 = d.extern_addr(1, gA), d0 = d.extern_addr(1, gB);
            DmaAddr s1 = d.extern_addr(1, gA + 0x400'0000), d1 = d.extern_addr(1, gB + 0x400'0000);
            auto t0 = std::chrono::steady_clock::now();
            d.start(0, s0, d0, sz, o0);
            d.start(1, s1, d1, sz, o1);
            XferResult r0 = d.wait(0, 10), r1 = d.wait(1, 10);
            double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            printf("  GDDR->GDDR EXT/EXT  2 ch x %5llu KiB      : %s %8.3f ms  %8.1f MiB/s aggregate\n", (unsigned long long)(sz >> 10),
                   (r0.ok && r1.ok) ? "ok  " : "FAIL", secs * 1e3, 2.0 * sz / secs / (1 << 20));
        }
        d.reset_tlbs_to_identity();
    }
    return 0;
}


// -----------------------------------------------------------------------------
// scrub: initialise L3 LIM ECC on all L2CPUs by running a hart (one-shot)
// limcheck: show LIM contents and DMA readability per 128 KiB
// -----------------------------------------------------------------------------

int cmd_scrub(Device& dev) {
    Noc noc(dev);
    printf("Scrubbing L3 LIM ECC on all L2CPUs by booting a hart in each (one-shot per ASIC reset)\n");
    unsigned done = scrub_l3_lim(noc);
    return done == 0xF ? 0 : 1;
}

int cmd_limcheck(Device& dev, int only) {
    Noc noc(dev);
    for (int i = 0; i < 4; i++) {
        if (only >= 0 && i != only) continue;
        Dmac d(noc, i);
        printf("L2CPU %d: WayEnable=%u\n", i, (unsigned)(d.phys_read64(l3::BASE + l3::WAY_ENABLE) & 0xFF));
        d.reset_tlbs_to_identity();
        const uint64_t limC = X280_LIM_BASE + 0x1C'0000;
        for (uint64_t off = 0; off < 2 * 1024 * 1024; off += 128 * 1024) {
            uint64_t w0 = d.phys_read64(X280_LIM_BASE + off), w1 = d.phys_read64(X280_LIM_BASE + off + 0x1000);
            XferResult r = d.copy(0, { Master::L2, (uint32_t)(X280_LIM_BASE + off) }, { Master::L2, (uint32_t)limC }, 4096, XferOptions());
            printf("  LIM+0x%06llx: host sees %016llx %016llx  DMA read %s\n", (unsigned long long)off,
                   (unsigned long long)w0, (unsigned long long)w1, r.ok ? "ok" : (r.timed_out ? "TIMEOUT" : "ERR"));
        }
    }
    return 0;
}

// nocread x y addr bytes ordering chain|split [burst]: host-driven DMA read from a NOC
// location through small window 0 into LIM, as one chained transfer or as
// separate single-block transfers.
int cmd_nocread(Device& dev, int argc, char** argv) {
    if (argc < 8) { fprintf(stderr, "nocread x y addr bytes ordering chain|split [burst]\n"); return 2; }
    uint16_t x = atoi(argv[2]), y = atoi(argv[3]);
    uint64_t addr = strtoull(argv[4], nullptr, 0), bytes = strtoull(argv[5], nullptr, 0);
    unsigned ordering = atoi(argv[6]); bool chain = std::string(argv[7]) == "chain";
    unsigned burst = argc > 8 ? atoi(argv[8]) : 256;
    Noc noc(dev);
    Dmac d(noc, 0);
    d.reset_tlbs_to_identity();
    const uint64_t lim = X280_LIM_BASE + 0x10'0000;
    uint64_t cfg = nocwin::SMALL_CFG;
    noc.write64(d.tile().x, d.tile().y, cfg, (addr & ~(nocwin::SMALL_SIZE - 1)) >> 21);
    noc.write32(d.tile().x, d.tile().y, cfg + 8, nocwin::props_lo(x, y, 0, 0, false, ordering, false, false, 0));
    noc.write32(d.tile().x, d.tile().y, cfg + 12, 0);
    DmaAddr src = d.extern_noc_small(2, 0, addr & (nocwin::SMALL_SIZE - 1));
    Pattern p(bytes, 55);
    noc.write(x, y, addr, p.bytes.data(), bytes);
    d.phys_fill64(lim, 0xEEEEEEEEEEEEEEEEULL, bytes);
    XferOptions o; o.timeout_s = 3; o.burst_items = burst;
    bool ok = true; double secs = 0;
    if (chain) {
        XferResult r = d.copy(0, src, { Master::L2, (uint32_t)lim }, bytes, o);
        ok = r.ok; secs = r.seconds;
        if (!r.ok) printf("  chained copy: %s%s\n", r.timed_out ? "TIMEOUT" : "", r.error_irq ? " ERR" : "");
    } else {
        const uint64_t blk = 4095ull * 32;
        for (uint64_t off = 0; off < bytes && ok; off += blk) {
            uint64_t len = std::min(blk, bytes - off);
            XferResult r = d.copy(0, { Master::Extern, (uint32_t)(src.addr + off) }, { Master::L2, (uint32_t)(lim + off) }, len, o);
            ok = r.ok; secs += r.seconds;
            if (!r.ok) printf("  block at %llu: %s%s\n", (unsigned long long)off, r.timed_out ? "TIMEOUT" : "", r.error_irq ? " ERR" : "");
        }
    }
    std::vector<uint8_t> back(bytes); d.phys_read(lim, back.data(), bytes);
    size_t bad = bytes; for (size_t i = 0; i < bytes; i++) if (back[i] != p.bytes[i]) { bad = i; break; }
    printf("  [%s] (%u,%u)+0x%llx -> LIM %llu B ordering=%u %s burst=%u  %.3f ms", (ok && bad == bytes) ? "PASS" : "FAIL", x, y,
           (unsigned long long)addr, (unsigned long long)bytes, ordering, chain ? "chain" : "split", burst, secs * 1e3);
    if (bad != bytes) printf("  mismatch @%zu", bad);
    printf("\n");
    d.reset_tlbs_to_identity();
    return 0;
}

std::unique_ptr<Device> open_blackhole() {
    for (auto& p : Device::enumerate()) {
        auto d = std::make_unique<Device>(p);
        if (d->is_blackhole()) {
            printf("Using %s (%s)\n", p.c_str(), d->bdf().c_str());
            return d;
        }
    }
    throw std::runtime_error("no Blackhole device found under /dev/tenstorrent");
}

} // namespace

int main(int argc, char** argv) {
    std::string cmd = argc > 1 ? argv[1] : "probe";
    int only = argc > 2 ? atoi(argv[2]) : -1;
    try {
        auto dev = open_blackhole();
        if (cmd == "probe") return cmd_probe(*dev);
        if (cmd == "test")  return cmd_test(*dev, only);
        if (cmd == "perf")  return cmd_perf(*dev, only, argc > 3 ? argv[3] : nullptr);
        if (cmd == "pll")   return cmd_pll(*dev, argc > 2 ? argv[2] : nullptr, argc > 3 ? atof(argv[3]) : 0.0);
        if (cmd == "nocread") return cmd_nocread(*dev, argc, argv);
        if (cmd == "scrub") return cmd_scrub(*dev);
        if (cmd == "limcheck") return cmd_limcheck(*dev, only);
        // Debug helpers: rd32|rd64 x y addr   wr32|wr64 x y addr value
        if ((cmd == "rd32" || cmd == "rd64") && argc == 5) {
            Noc noc(*dev);
            uint16_t x = atoi(argv[2]), y = atoi(argv[3]); uint64_t a = strtoull(argv[4], nullptr, 0);
            if (cmd == "rd32") printf("(%u,%u) 0x%016" PRIx64 " = 0x%08x\n", x, y, a, noc.read32(x, y, a));
            else               printf("(%u,%u) 0x%016" PRIx64 " = 0x%016" PRIx64 "\n", x, y, a, noc.read64(x, y, a));
            return 0;
        }
        if ((cmd == "wr32" || cmd == "wr64") && argc == 6) {
            Noc noc(*dev);
            uint16_t x = atoi(argv[2]), y = atoi(argv[3]); uint64_t a = strtoull(argv[4], nullptr, 0), v = strtoull(argv[5], nullptr, 0);
            if (cmd == "wr32") noc.write32(x, y, a, (uint32_t)v); else noc.write64(x, y, a, v);
            printf("wrote\n");
            return 0;
        }
        fprintf(stderr,
                "usage: %s <command> [args]\n"
                "  probe              read-only register dump of all four L2CPU DMA engines\n"
                "  scrub              ECC-scrub the L3 LIM of all four tiles by booting a hart in each (one-shot per ASIC reset)\n"
                "  limcheck [l2cpu]   show LIM contents and whether the DMA can read each 128 KiB\n"
                "  test [l2cpu]       functional tests (LIM, GDDR, linked lists, both channels, NOC window to Tensix)\n"
                "  perf [l2cpu] [mhz] throughput sweep, optionally after setting the PLL (200|800|1750)\n"
                "  pll [mhz] [secs]   show or set the L2CPU PLL, optionally hold the device open and watch it\n"
                "  nocread x y addr bytes ordering chain|split [burst]   host-driven DMA read of a NOC location into LIM\n"
                "  rd32|rd64 x y addr          raw NOC read\n"
                "  wr32|wr64 x y addr value    raw NOC write\n", argv[0]);
        return 2;
    } catch (const std::exception& e) {
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
