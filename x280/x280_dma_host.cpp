// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// x280_dma_host.cpp - boot a Blackhole L2CPU with dma_fw and have the X280
// itself drive the tile's DMA engine: LIM, local GDDR, a Tensix tile's L1,
// a DRAM tile over the NOC, and a pinned host buffer over NOC/PCIe.
//
// Self-contained apart from the firmware: the tt-kmd ioctl ABI is inlined,
// and the image (fw/dma_fw.bin, built from fw/dma_fw.c) is read at run time,
// by default from fw/dma_fw.bin next to this executable.
//
//   ./x280_dma_host [l2cpu] [--keep] [--fw <dma_fw.bin>]
//                     default L2CPU 0; --keep leaves the firmware serving
//                     commands for `attach` instead of parking it
//   ./x280_dma_host <l2cpu> attach <cmd ...>
//
// The clusters can be released from reset only once per ASIC reset, so run
// `tt-smi -r` first. The firmware ECC-scrubs the LIM itself on boot.

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <dirent.h>
#include <random>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include <fcntl.h>
#include <linux/types.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace {

// ============================================================================
// Inlined tt-kmd ABI
// ============================================================================

constexpr unsigned TT_IOCTL_MAGIC = 0xFA;
#define TT_IO(nr) _IO(TT_IOCTL_MAGIC, nr)
constexpr unsigned long IOCTL_GET_DEVICE_INFO = TT_IO(0);
constexpr unsigned long IOCTL_PIN_PAGES       = TT_IO(7);
constexpr unsigned long IOCTL_UNPIN_PAGES     = TT_IO(10);
constexpr unsigned long IOCTL_ALLOCATE_TLB    = TT_IO(11);
constexpr unsigned long IOCTL_FREE_TLB        = TT_IO(12);
constexpr unsigned long IOCTL_CONFIGURE_TLB   = TT_IO(13);
#undef TT_IO
constexpr uint16_t BLACKHOLE_PCI_DEVICE_ID = 0xb140;

struct tt_get_device_info_in  { __u32 output_size_bytes; };
struct tt_get_device_info_out { __u32 output_size_bytes; __u16 vendor_id, device_id, subsystem_vendor_id, subsystem_id;
                                __u16 bus_dev_fn, max_dma_buf_size_log2, pci_domain, reserved; };
struct tt_get_device_info     { tt_get_device_info_in in; tt_get_device_info_out out; };
struct tt_allocate_tlb_in     { __u64 size; __u64 reserved; };
struct tt_allocate_tlb_out    { __u32 id; __u32 reserved0; __u64 mmap_offset_uc; __u64 mmap_offset_wc; __u64 reserved1; };
struct tt_allocate_tlb        { tt_allocate_tlb_in in; tt_allocate_tlb_out out; };
struct tt_free_tlb_in         { __u32 id; };
struct tt_free_tlb            { tt_free_tlb_in in; };
struct tt_noc_tlb_config      { __u64 addr; __u16 x_end, y_end, x_start, y_start; __u8 noc, mcast, ordering, linked, static_vc;
                                __u8 reserved0[3]; __u32 reserved1[2]; };
struct tt_configure_tlb_in    { __u32 id; __u32 reserved; tt_noc_tlb_config config; };
struct tt_configure_tlb_out   { __u64 reserved; };
struct tt_configure_tlb       { tt_configure_tlb_in in; tt_configure_tlb_out out; };
// PIN_PAGES: with the IOMMU on, physical_address is the IOVA, and the IOVA is
// directly usable as a NOC address at the PCIe tile. No NOC_DMA flag needed.
struct tt_pin_pages_in        { __u32 output_size_bytes; __u32 flags; __u64 virtual_address; __u64 size; };
struct tt_pin_pages_out       { __u64 physical_address; };
struct tt_pin_pages           { tt_pin_pages_in in; tt_pin_pages_out out; };
struct tt_unpin_pages_in      { __u64 virtual_address; __u64 size; __u64 reserved; };
struct tt_unpin_pages         { tt_unpin_pages_in in; };

// ============================================================================
// NOC access through a 2 MiB PCIe->NOC window (layer 1 of the three TLBs)
// ============================================================================

constexpr uint64_t TLB_SIZE = 1ULL << 21;

struct TlbWindow {
    int fd; uint32_t id = 0; void* base = MAP_FAILED; bool mapped = false; uint16_t cx = 0, cy = 0; uint64_t cbase = 0;
    TlbWindow(int fd_, bool wc) : fd(fd_) {
        tt_allocate_tlb a = {}; a.in.size = TLB_SIZE;
        if (ioctl(fd, IOCTL_ALLOCATE_TLB, &a) != 0) throw std::system_error(errno, std::generic_category(), "ALLOCATE_TLB");
        id = a.out.id;
        base = mmap(nullptr, TLB_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, (off_t)(wc ? a.out.mmap_offset_wc : a.out.mmap_offset_uc));
        if (base == MAP_FAILED) throw std::system_error(errno, std::generic_category(), "mmap TLB");
    }
    ~TlbWindow() { if (base != MAP_FAILED) munmap(base, TLB_SIZE); tt_free_tlb f = {}; f.in.id = id; ioctl(fd, IOCTL_FREE_TLB, &f); }
    volatile uint8_t* ptr(uint16_t x, uint16_t y, uint64_t addr) {
        uint64_t aligned = addr & ~(TLB_SIZE - 1);
        if (!mapped || x != cx || y != cy || aligned != cbase) {
            tt_configure_tlb c = {}; c.in.id = id; c.in.config.addr = aligned; c.in.config.x_end = x; c.in.config.y_end = y;
            if (ioctl(fd, IOCTL_CONFIGURE_TLB, &c) != 0) throw std::system_error(errno, std::generic_category(), "CONFIGURE_TLB");
            mapped = true; cx = x; cy = y; cbase = aligned;
        }
        return (volatile uint8_t*)base + (addr & (TLB_SIZE - 1));
    }
};

struct Noc {
    TlbWindow uc, wc;
    explicit Noc(int fd) : uc(fd, false), wc(fd, true) {}
    uint32_t read32(uint16_t x, uint16_t y, uint64_t a)            { return *(volatile uint32_t*)uc.ptr(x, y, a); }
    void     write32(uint16_t x, uint16_t y, uint64_t a, uint32_t v){ *(volatile uint32_t*)uc.ptr(x, y, a) = v; }
    uint64_t read64(uint16_t x, uint16_t y, uint64_t a)            { return *(volatile uint64_t*)uc.ptr(x, y, a); }
    void     write64(uint16_t x, uint16_t y, uint64_t a, uint64_t v){ *(volatile uint64_t*)uc.ptr(x, y, a) = v; }
    void write(uint16_t x, uint16_t y, uint64_t a, const void* src, size_t len) {
        const uint8_t* s = (const uint8_t*)src;
        while (len) {
            size_t chunk = std::min<size_t>(len, TLB_SIZE - (a & (TLB_SIZE - 1)));
            volatile uint64_t* d = (volatile uint64_t*)wc.ptr(x, y, a);
            for (size_t i = 0; i < chunk / 8; i++) { uint64_t v; memcpy(&v, s + 8 * i, 8); d[i] = v; }
            s += chunk; a += chunk; len -= chunk;
        }
        __sync_synchronize();
    }
    void read(uint16_t x, uint16_t y, uint64_t a, void* dst, size_t len) {
        uint8_t* d = (uint8_t*)dst;
        while (len) {
            size_t chunk = std::min<size_t>(len, TLB_SIZE - (a & (TLB_SIZE - 1)));
            volatile uint64_t* s = (volatile uint64_t*)uc.ptr(x, y, a);
            for (size_t i = 0; i < chunk / 8; i++) { uint64_t v = s[i]; memcpy(d + 8 * i, &v, 8); }
            d += chunk; a += chunk; len -= chunk;
        }
    }
};

// ============================================================================
// Blackhole geometry and addresses
// ============================================================================

struct Tile { uint16_t x, y; };
constexpr Tile L2CPU_TILE[4] = { {8, 3}, {8, 9}, {8, 5}, {8, 7} };
constexpr Tile ARC    = { 8, 0 };
constexpr Tile PCIE   = { 19, 24 };   // the PCIe tile connected to the host
constexpr Tile TENSIX = { 1, 2 };     // any Tensix tile; 1.5 MiB of L1 at address 0
constexpr Tile DRAM   = { 17, 12 };   // a DRAM tile reached over the NOC, 4 GiB at address 0

constexpr uint64_t PERIPH_BASE      = 0xFFFF'F7FE'FFF0'0000ULL;   // bypass alias of X280 phys 0x2000_0000
constexpr uint64_t L2CPU_REG_BASE   = PERIPH_BASE + 0x1'0000;
constexpr uint64_t REG_RESET_VECTOR = L2CPU_REG_BASE + 0x000;     // 8 bytes per hart
constexpr uint64_t REG_X280_MGMT    = L2CPU_REG_BASE + 0x400;     // [19:16] suppress fetch
constexpr uint64_t ARC_L2CPU_RESET  = 0x8003'0014ULL;
constexpr uint64_t PLL4_CNTL_1      = 0x8002'0504ULL;
constexpr uint64_t PLL4_CNTL_5      = 0x8002'0514ULL;

constexpr uint64_t X280_LIM_BASE     = 0x0000'0800'0000ULL;
constexpr uint64_t X280_GDDR_UNCACHED= 0x0000'3000'0000ULL;
constexpr uint64_t X280_GDDR_CACHED  = 0x4000'3000'0000ULL;

constexpr size_t FW_MAX_SIZE = 0x1'0000;   // image + stack, enforced by fw/link.ld

// Firmware image at GDDR offset 3.75 GiB (+16 MiB per tile: tiles 2 and 3
// share a DRAM), code through the cached alias, mailbox 64 KiB later through
// the uncached alias.
uint64_t fw_gddr_offset(int i) { return 0xF000'0000ULL + (uint64_t)i * 0x100'0000; }
uint64_t fw_code_phys(int i)   { return X280_GDDR_CACHED + fw_gddr_offset(i); }
uint64_t fw_mbox_phys(int i)   { return X280_GDDR_UNCACHED + fw_gddr_offset(i) + FW_MAX_SIZE; }

// Mailbox layout, mirrors dma_fw.c.
constexpr uint64_t MBOX_MAGIC = 0x58323830444D4131ULL;
enum { MB_MAGIC = 0x00, MB_STATE = 0x08, MB_CMD = 0x10, MB_SEQ = 0x18, MB_STATUS = 0x20, MB_CYCLES = 0x28,
       MB_TRAP_CAUSE = 0x30, MB_TRAP_EPC = 0x38, MB_TRAP_TVAL = 0x40, MB_ARGS = 0x80, MB_RES = 0x100, MB_SIZE = 0x180 };
enum { ST_BOOT = 0, ST_SCRUB = 1, ST_READY = 2, ST_BUSY = 3, ST_PARKED = 4, ST_TRAPPED = 0xDEAD };
enum { CMD_NOP = 1, CMD_DMA = 2, CMD_RD64 = 3, CMD_WR64 = 4, CMD_DUMP = 5, CMD_PARK = 6 };
enum { KIND_PHYS = 0, KIND_NOC_SMALL = 1, KIND_NOC_LARGE = 2, KIND_XBAR = 3 };
const char* status_name(uint64_t s) {
    static const char* n[] = { "ok", "bad-arg", "range", "timeout", "dma-error", "no-tfr-irq", "unknown-cmd" };
    return s < 7 ? n[s] : "?";
}

// ============================================================================
// PLL stepping (from x280_tester / tt-bh-linux clock.py)
// ============================================================================

struct PllConfig { uint16_t fbdiv; uint8_t postdiv[4]; };
constexpr PllConfig PLL_200MHZ  = { 128, { 15, 15, 15, 15 } };
constexpr PllConfig PLL_1750MHZ = { 140, {  1,  1,  1,  1 } };

void set_l2cpu_pll(Noc& noc, const PllConfig& t) {
    uint32_t c1 = noc.read32(ARC.x, ARC.y, PLL4_CNTL_1), c5 = noc.read32(ARC.x, ARC.y, PLL4_CNTL_5);
    uint16_t fb = c1 >> 16;
    uint8_t pd[4] = { (uint8_t)c5, (uint8_t)(c5 >> 8), (uint8_t)(c5 >> 16), (uint8_t)(c5 >> 24) };
    auto wpd = [&] { noc.write32(ARC.x, ARC.y, PLL4_CNTL_5, pd[0] | pd[1] << 8 | pd[2] << 16 | pd[3] << 24); };
    for (int i = 0; i < 4; i++) while (pd[i] < t.postdiv[i]) { pd[i]++; wpd(); }
    while (fb != t.fbdiv) { fb += fb < t.fbdiv ? 1 : -1; noc.write32(ARC.x, ARC.y, PLL4_CNTL_1, (c1 & 0xFFFF) | (uint32_t)fb << 16); }
    for (int i = 0; i < 4; i++) while (pd[i] > t.postdiv[i]) { pd[i]--; wpd(); }
}
double l2cpu_mhz(Noc& noc) {
    uint32_t c1 = noc.read32(ARC.x, ARC.y, PLL4_CNTL_1), c5 = noc.read32(ARC.x, ARC.y, PLL4_CNTL_5);
    return (c1 >> 16) * 12.5 / std::max<unsigned>(1, c5 & 0xFF);
}

// ============================================================================
// Firmware boot and mailbox
// ============================================================================

struct X280 {
    Noc& noc; int idx; Tile t; uint64_t mbox;
    X280(Noc& n, int i) : noc(n), idx(i), t(L2CPU_TILE[i]), mbox(fw_mbox_phys(i)) {}

    uint64_t mb_rd(unsigned off)             { return noc.read64(t.x, t.y, mbox + off); }
    void     mb_wr(unsigned off, uint64_t v) { noc.write64(t.x, t.y, mbox + off, v); }

    void boot(const std::vector<uint8_t>& fw) {
        uint32_t reset = noc.read32(ARC.x, ARC.y, ARC_L2CPU_RESET);
        if ((reset >> 4) & (1u << idx))
            throw std::runtime_error("L2CPU " + std::to_string(idx) + " was already released from reset; harts can only be "
                                     "booted once per ASIC reset. Run `tt-smi -r` and retry.");
        printf("  PLL -> 200 MHz for the release\n");
        set_l2cpu_pll(noc, PLL_200MHZ);

        uint64_t code = fw_code_phys(idx);
        printf("  loading dma_fw (%zu bytes) at X280 phys 0x%" PRIx64 " (cached GDDR alias), mailbox at 0x%" PRIx64 "\n",
               fw.size(), code, mbox);
        noc.write(t.x, t.y, code, fw.data(), fw.size());
        for (unsigned off = 0; off < MB_SIZE; off += 8) mb_wr(off, 0);
        for (int hart = 0; hart < 4; hart++) {
            noc.write32(t.x, t.y, REG_RESET_VECTOR + hart * 8,     (uint32_t)code);
            noc.write32(t.x, t.y, REG_RESET_VECTOR + hart * 8 + 4, (uint32_t)(code >> 32));
        }
        uint32_t mgmt = noc.read32(t.x, t.y, REG_X280_MGMT);
        if (mgmt >> 16) noc.write32(t.x, t.y, REG_X280_MGMT, mgmt & 0xFFFF);

        printf("  releasing cluster %d (L2CPU_RESET 0x%02x -> 0x%02x)\n", idx, reset, reset | (0x10u << idx));
        noc.write32(ARC.x, ARC.y, ARC_L2CPU_RESET, reset | (0x10u << idx));

        wait_for([&] { return mb_rd(MB_MAGIC) == MBOX_MAGIC; }, 5.0, "firmware did not come up (no magic in mailbox)");
        printf("  firmware alive; PLL -> 1750 MHz\n");
        set_l2cpu_pll(noc, PLL_1750MHZ);
        wait_for([&] { return mb_rd(MB_STATE) == ST_READY; }, 20.0, "firmware did not finish the LIM scrub");
        printf("  LIM ECC scrub done by hart 0 in %.1f ms\n", mb_rd(MB_CYCLES) / (l2cpu_mhz(noc) * 1e3));
    }

    template <class F> void wait_for(F f, double timeout_s, const char* what) {
        auto t0 = std::chrono::steady_clock::now();
        while (!f()) {
            uint64_t st = mb_rd(MB_STATE);
            if (st == ST_TRAPPED)
                throw std::runtime_error("firmware trapped: mcause=0x" + hex(mb_rd(MB_TRAP_CAUSE)) + " mepc=0x" + hex(mb_rd(MB_TRAP_EPC)) +
                                         " mtval=0x" + hex(mb_rd(MB_TRAP_TVAL)));
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeout_s)
                throw std::runtime_error(std::string(what) + " (state=" + std::to_string(st) + ")");
            std::this_thread::sleep_for(std::chrono::microseconds(50));
        }
    }
    static std::string hex(uint64_t v) { char b[32]; snprintf(b, sizeof(b), "%" PRIx64, v); return b; }

    struct Reply { uint64_t status, cycles, res[16]; };
    Reply command(uint64_t cmd, const std::vector<uint64_t>& args = {}, double timeout_s = 15.0) {
        for (size_t i = 0; i < 16; i++) mb_wr(MB_ARGS + 8 * i, i < args.size() ? args[i] : 0);
        uint64_t seq = mb_rd(MB_SEQ);
        mb_wr(MB_CMD, cmd);
        wait_for([&] { return mb_rd(MB_SEQ) != seq && mb_rd(MB_CMD) == 0; }, timeout_s, "firmware did not answer");
        Reply r; r.status = mb_rd(MB_STATUS); r.cycles = mb_rd(MB_CYCLES);
        for (size_t i = 0; i < 16; i++) r.res[i] = mb_rd(MB_RES + 8 * i);
        return r;
    }

    void dump_dmac() {
        Reply r = command(CMD_DUMP);
        printf("    DMAC: COMP_ID %016" PRIx64 " DMA_ID %" PRIx64 " DMA_CFG %" PRIx64 " CH_EN %" PRIx64 " RAW_TFR %" PRIx64 " RAW_ERR %" PRIx64 "\n",
               r.res[0], r.res[1], r.res[2], r.res[3], r.res[4], r.res[5]);
        for (int ch = 0; ch < 2; ch++)
            printf("    ch%d: SAR %08" PRIx64 " DAR %08" PRIx64 " LLP %08" PRIx64 " CTL %016" PRIx64 " CFG %016" PRIx64 "\n", ch,
                   r.res[6 + 5 * ch], r.res[7 + 5 * ch], r.res[8 + 5 * ch], r.res[9 + 5 * ch], r.res[10 + 5 * ch]);
    }
};

// ============================================================================
// Test harness: host prepares the source, X280 DMAs, host verifies the target
// ============================================================================

struct Endpoint { uint64_t kind; Tile tile; uint64_t addr; };   // kind per dma_fw.c
struct HostView { Tile tile; uint64_t addr; uint8_t* host = nullptr; };   // how the host reads/writes that endpoint

Endpoint phys(uint64_t a)                     { return { KIND_PHYS, {0, 0}, a }; }
Endpoint noc_small(Tile t, uint64_t a)        { return { KIND_NOC_SMALL, t, a }; }
Endpoint noc_large(Tile t, uint64_t a)        { return { KIND_NOC_LARGE, t, a }; }
Endpoint xbar(uint64_t a)                     { return { KIND_XBAR, {0, 0}, a }; }

int g_pass = 0, g_fail = 0;

void host_write(Noc& noc, const HostView& v, const std::vector<uint8_t>& data) {
    if (v.host) memcpy(v.host, data.data(), data.size()); else noc.write(v.tile.x, v.tile.y, v.addr, data.data(), data.size());
}
void host_read(Noc& noc, const HostView& v, std::vector<uint8_t>& data) {
    if (v.host) { __sync_synchronize(); memcpy(data.data(), v.host, data.size()); } else noc.read(v.tile.x, v.tile.y, v.addr, data.data(), data.size());
}

bool run_case(Noc& noc, X280& x, const char* name, Endpoint src, Endpoint dst, HostView sv, HostView dv, size_t bytes,
              unsigned width = 32, unsigned burst = 256, unsigned ch = 0) {
    std::vector<uint8_t> pattern(bytes), back(bytes);
    std::mt19937_64 rng(bytes ^ (uint64_t)name[0] << 32 ^ src.addr ^ dst.addr);
    for (size_t i = 0; i < bytes; i += 8) { uint64_t v = rng(); memcpy(&pattern[i], &v, std::min<size_t>(8, bytes - i)); }
    std::vector<uint8_t> poison(bytes, 0xEE);
    host_write(noc, sv, pattern);
    host_write(noc, dv, poison);

    X280::Reply r = x.command(CMD_DMA, { src.kind, src.tile.x, src.tile.y, src.addr, dst.kind, dst.tile.x, dst.tile.y, dst.addr,
                                         bytes, width, burst, ch });
    host_read(noc, dv, back);
    size_t bad = bytes;
    for (size_t i = 0; i < bytes; i++) if (back[i] != pattern[i]) { bad = i; break; }
    bool ok = r.status == 0 && bad == bytes;
    double secs = r.cycles / (l2cpu_mhz(noc) * 1e6);
    printf("  [%s] %-46s %4zu KiB  %u blk  %8.3f ms  %7.1f MiB/s", ok ? "PASS" : "FAIL", name, bytes >> 10, (unsigned)r.res[0],
           secs * 1e3, bytes / secs / (1 << 20));
    if (r.status) printf("  status=%s raw_err=%" PRIx64, status_name(r.status), r.res[1]);
    if (bad != bytes) printf("  mismatch @%zu", bad);
    printf("\n");
    if (!ok) x.dump_dmac();
    (ok ? g_pass : g_fail)++;
    return ok;
}

// Pinned host buffer: page-aligned anonymous memory pinned for device access.
// With the IOMMU enabled the returned address is an IOVA, which is a valid
// NOC address at the PCIe tile.
struct PinnedBuffer {
    int fd; size_t size; uint8_t* va = nullptr; uint64_t iova = 0;
    PinnedBuffer(int fd_, size_t sz) : fd(fd_), size(sz) {
        void* p = mmap(nullptr, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, -1, 0);
        if (p == MAP_FAILED) throw std::system_error(errno, std::generic_category(), "mmap host buffer");
        va = (uint8_t*)p;
        memset(va, 0, size);
        tt_pin_pages pin = {};
        pin.in.output_size_bytes = sizeof(pin.out);
        pin.in.flags = 0;
        pin.in.virtual_address = (uint64_t)(uintptr_t)va;
        pin.in.size = size;
        if (ioctl(fd, IOCTL_PIN_PAGES, &pin) != 0) { int e = errno; munmap(va, size); throw std::system_error(e, std::generic_category(), "PIN_PAGES"); }
        iova = pin.out.physical_address;
    }
    ~PinnedBuffer() {
        tt_unpin_pages un = {}; un.in.virtual_address = (uint64_t)(uintptr_t)va; un.in.size = size;
        ioctl(fd, IOCTL_UNPIN_PAGES, &un);
        munmap(va, size);
    }
};

// Default firmware location: fw/dma_fw.bin next to the executable, which is
// where the Makefiles put it.
std::string default_fw_path() {
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "fw/dma_fw.bin";
    std::string exe(buf, n);
    return exe.substr(0, exe.rfind('/') + 1) + "fw/dma_fw.bin";
}

// Reads the raw image, padded to a multiple of 8 bytes (Noc::write moves
// whole 64-bit words).
std::vector<uint8_t> load_firmware(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) throw std::system_error(errno, std::generic_category(), "open firmware " + path + " (build it with `make fw` at the top of the repo)");
    std::vector<uint8_t> img;
    uint8_t buf[4096];
    for (size_t n; (n = fread(buf, 1, sizeof(buf), f)) > 0;) img.insert(img.end(), buf, buf + n);
    fclose(f);
    if (img.empty() || img.size() > FW_MAX_SIZE)
        throw std::runtime_error("firmware " + path + " is " + std::to_string(img.size()) + " bytes; expected 1.." + std::to_string(FW_MAX_SIZE));
    img.resize((img.size() + 7) & ~size_t(7), 0);
    return img;
}

std::string find_blackhole() {
    std::vector<std::pair<int, std::string>> found;
    if (DIR* d = opendir("/dev/tenstorrent")) {
        while (struct dirent* e = readdir(d))
            if (*e->d_name && strspn(e->d_name, "0123456789") == strlen(e->d_name)) found.emplace_back(atoi(e->d_name), std::string("/dev/tenstorrent/") + e->d_name);
        closedir(d);
    }
    std::sort(found.begin(), found.end());
    for (auto& f : found) {
        int fd = open(f.second.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) continue;
        tt_get_device_info info = {}; info.in.output_size_bytes = sizeof(info.out);
        bool bh = ioctl(fd, IOCTL_GET_DEVICE_INFO, &info) == 0 && info.out.device_id == BLACKHOLE_PCI_DEVICE_ID;
        close(fd);
        if (bh) return f.second;
    }
    throw std::runtime_error("no Blackhole under /dev/tenstorrent");
}

} // namespace

// Attach to an already running firmware and issue one command:
//   dma <src_kind> <sx> <sy> <saddr> <dst_kind> <dx> <dy> <daddr> <bytes>   (kinds: 0 phys, 1 noc-small, 2 noc-large, 3 xbar)
//   rd64 <phys> | wr64 <phys> <val> | dump | park
int attach_main(Noc& noc, X280& x, int argc, char** argv) {
    if (x.mb_rd(MB_MAGIC) != MBOX_MAGIC) throw std::runtime_error("no running firmware on this L2CPU (boot it first)");
    std::string c = argc > 3 ? argv[3] : "";
    auto u = [&](int i) { return (uint64_t)strtoull(argv[i], nullptr, 0); };
    X280::Reply r = {};
    if (c == "dma" && argc == 13) {
        r = x.command(CMD_DMA, { u(4), u(5), u(6), u(7), u(8), u(9), u(10), u(11), u(12), 32, 256, 0 }, 20.0);
        printf("status=%s blocks=%" PRIu64 " raw_err=%" PRIx64 " raw_tfr=%" PRIx64 " cycles=%" PRIu64 " (%.3f ms)\n",
               status_name(r.status), r.res[0], r.res[1], r.res[2], r.cycles, r.cycles / (l2cpu_mhz(noc) * 1e3));
    } else if (c == "rd64" && argc == 5) {
        r = x.command(CMD_RD64, { u(4) });
        printf("0x%016" PRIx64 "\n", r.res[0]);
    } else if (c == "wr64" && argc == 6) {
        r = x.command(CMD_WR64, { u(4), u(5) });
        printf("status=%s\n", status_name(r.status));
    } else if (c == "hostprobe" && (argc == 6 || argc == 7)) {
        uint64_t kind = argc == 7 ? u(6) : (uint64_t)KIND_NOC_LARGE;   // 1 = small window, 2 = large window
        // Pin a host buffer and have the X280 DMA 64 B of LIM into it through a
        // large window at (x, y); reports whether the bytes arrived.
        PinnedBuffer buf(x.noc.uc.fd, 2 << 20);
        uint64_t tag = 0xB0B0000000000000ULL | (u(4) << 16) | u(5);
        x.noc.write64(x.t.x, x.t.y, X280_LIM_BASE + 0x10'0000, tag);
        memset(buf.va, 0, 64);
        r = x.command(CMD_DMA, { KIND_PHYS, 0, 0, X280_LIM_BASE + 0x10'0000, kind, u(4), u(5), buf.iova, 64, 32, 256, 0 }, 10.0);
        __sync_synchronize();
        uint64_t got; memcpy(&got, buf.va, 8);
        printf("(%" PRIu64 ",%" PRIu64 ") iova 0x%" PRIx64 ": status=%s host sees 0x%016" PRIx64 " %s\n", u(4), u(5), buf.iova,
               status_name(r.status), got, got == tag ? "<-- ARRIVED" : "");
    } else if (c == "dump") {
        x.dump_dmac();
    } else if (c == "park") {
        x.command(CMD_PARK);
    } else {
        fprintf(stderr, "attach: dma <sk> <sx> <sy> <sa> <dk> <dx> <dy> <da> <bytes> | rd64 <a> | wr64 <a> <v> | dump | park\n");
        return 2;
    }
    return r.status ? 1 : 0;
}

int main(int argc, char** argv) {
    // Pull out --fw <path>; the remaining arguments are positional.
    std::string fw_path = default_fw_path();
    for (int i = 1; i < argc; i++) {
        if (std::string(argv[i]) != "--fw") continue;
        if (i + 1 >= argc) { fprintf(stderr, "--fw needs a path\n"); return 2; }
        fw_path = argv[i + 1];
        for (int j = i; j + 2 <= argc; j++) argv[j] = argv[j + 2];
        argc -= 2; i--;
    }
    int idx = argc > 1 ? atoi(argv[1]) : 0;
    if (idx < 0 || idx > 3) { fprintf(stderr, "usage: %s [l2cpu 0..3] [--keep] [--fw <dma_fw.bin>] | <l2cpu> attach <command...>\n", argv[0]); return 2; }
    try {
        bool attach = argc > 2 && std::string(argv[2]) == "attach";
        std::vector<uint8_t> fw;
        if (!attach) fw = load_firmware(fw_path);   // before touching the device, so a missing image costs nothing

        std::string path = find_blackhole();
        int fd = open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd < 0) throw std::system_error(errno, std::generic_category(), "open " + path);
        printf("Using %s, L2CPU %d (%u,%u)\n", path.c_str(), idx, L2CPU_TILE[idx].x, L2CPU_TILE[idx].y);
        Noc noc(fd);
        X280 x(noc, idx);
        const Tile t = x.t;
        if (attach) return attach_main(noc, x, argc, argv);

        printf("Booting %s\n", fw_path.c_str());
        x.boot(fw);

        // Locations. All X280-physical addresses below are also host-visible
        // through the L2CPU tile's NOC passthrough at the same address. Usable
        // LIM ends at 0x081E_0000 and the firmware keeps its descriptors in the
        // last 64 KiB, so LIM destinations must stay below 0x081D_0000: a DMA
        // write past the end of the LIM hangs the channel.
        const uint64_t limA = X280_LIM_BASE + 0x10'0000, limB = X280_LIM_BASE + 0x14'0000;
        const uint64_t gA = X280_GDDR_UNCACHED + (1ULL << 30), gB = gA + 0x100'0000;   // local GDDR, 1 GiB in
        const uint64_t l1 = 0x10'0000;                                                 // 1 MiB into Tensix L1
        const uint64_t dram = 0x1000'0000;                                             // 256 MiB into the DRAM tile
        HostView hv_limA{t, limA}, hv_limB{t, limB}, hv_gA{t, gA}, hv_gB{t, gB}, hv_l1{TENSIX, l1}, hv_dram{DRAM, dram};

        printf("1. LIM and local GDDR (X280 physical addresses, DMA master 1)\n");
        run_case(noc, x, "LIM -> LIM", phys(limA), phys(limB), hv_limA, hv_limB, 256 << 10);
        run_case(noc, x, "LIM -> GDDR", phys(limA), phys(gA), hv_limA, hv_gA, 64 << 10);
        run_case(noc, x, "GDDR -> LIM", phys(gA), phys(limB), hv_gA, hv_limB, 64 << 10);
        run_case(noc, x, "GDDR -> GDDR (crossbar, master 0)", xbar(1ULL << 30), xbar((1ULL << 30) + 0x100'0000), hv_gA, hv_gB, 1 << 20);

        printf("2. Tensix (%u,%u) L1 through a 2 MiB X280 NOC window\n", TENSIX.x, TENSIX.y);
        run_case(noc, x, "GDDR -> Tensix L1", phys(gA), noc_small(TENSIX, l1), hv_gA, hv_l1, 256 << 10);
        run_case(noc, x, "Tensix L1 -> LIM", noc_small(TENSIX, l1), phys(limB), hv_l1, hv_limB, 256 << 10);

        printf("3. DRAM tile (%u,%u) over the NOC through a 2 MiB X280 NOC window\n", DRAM.x, DRAM.y);
        run_case(noc, x, "GDDR -> DRAM tile via NOC", phys(gA), noc_small(DRAM, dram), hv_gA, hv_dram, 1 << 20);
        run_case(noc, x, "DRAM tile via NOC -> LIM", noc_small(DRAM, dram), phys(limB), hv_dram, hv_limB, 512 << 10);
        run_case(noc, x, "Tensix L1 -> DRAM tile via NOC (NOC to NOC)", noc_small(TENSIX, l1), noc_small(DRAM, dram), hv_l1, hv_dram, 256 << 10);

        printf("4. Pinned host buffer through the PCIe tile (%u,%u) and a 128 GiB X280 NOC window\n", PCIE.x, PCIE.y);
        {
            PinnedBuffer buf(fd, 2 << 20);
            printf("  host buffer VA %p, IOVA 0x%" PRIx64 " (passed to the X280 through the mailbox)\n", (void*)buf.va, buf.iova);
            HostView hv_host{PCIE, buf.iova, buf.va};
            run_case(noc, x, "host -> GDDR", noc_large(PCIE, buf.iova), phys(gB), hv_host, hv_gB, 1 << 20);
            run_case(noc, x, "GDDR -> host", phys(gA), noc_large(PCIE, buf.iova), hv_gA, hv_host, 1 << 20);
            run_case(noc, x, "host -> Tensix L1", noc_large(PCIE, buf.iova), noc_small(TENSIX, l1), hv_host, hv_l1, 256 << 10);
            run_case(noc, x, "host -> host (through the X280 DMA)", noc_large(PCIE, buf.iova), noc_large(PCIE, buf.iova + (1 << 20)),
                     hv_host, HostView{PCIE, buf.iova + (1 << 20), buf.va + (1 << 20)}, 1 << 20);
        }

        bool keep = argc > 2 && std::string(argv[2]) == "--keep";
        if (keep) printf("%d passed, %d failed. Firmware left running: `%s %d attach ...` issues further commands.\n", g_pass, g_fail, argv[0], idx);
        else { x.command(CMD_PARK); printf("%d passed, %d failed; hart parked in wfi (use --keep to leave it serving commands)\n", g_pass, g_fail); }
        close(fd);
        return g_fail ? 1 : 0;
    } catch (const std::exception& e) {
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
