// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// l2cpu_dma.hpp - host-side driver for the DMA engine inside each Blackhole
// L2CPU tile, driven entirely over PCIe/NOC through tt-kmd. Single header, no
// dependencies beyond a C++17 compiler and the tt-kmd character device.
//
// =============================================================================
// What the hardware is
// =============================================================================
//
// Each L2CPU tile (4 x SiFive X280 harts + L2/L3) contains a Synopsys
// DW_ahb_dmac with:
//   * 2 channels
//   * a 256-bit datapath (transfer width up to 32 bytes)
//   * up to 4095 "words" per block (BLOCK_TS is 12 bits), so <=128 KiB per
//     block at 32-byte width; bigger moves use linked-list block chaining
//   * 64-bit wide registers in a 4 KiB window
//   * no hardware handshake lines (they are tied off): memory-to-memory only
//
// The controller has two AHB master ports, each with a 32-bit address that is
// widened by a small table of "DMA TLB" registers living in the L2CPU
// register block:
//
//   Master 0 ("EXTERN")  addr[31:28] picks one of 16 OUTBOUND_DMA_TLB regs
//                        (20 bits each). Result = {tlb[19:0], addr[27:0]}, a
//                        48-bit address into the tile's DDR/NOC crossbar.
//                        That crossbar space is the X280 "system port" space
//                        with 0x3000_0000 subtracted:
//                            0x0_0000_0000  local GDDR (4 GiB)
//                            0x4_0000_0000  small NOC TLB windows (224 x 2 MiB)
//                            0x804_0000_0000 large NOC TLB windows (32 x 128 GiB)
//                        Granularity: 256 MiB per TLB entry.
//
//   Master 1 ("L2")      addr[31:30] picks one of 4 INBOUND_DMA_TLB regs
//                        (17 bits each). Result = {tlb[16:0], addr[29:0]}, a
//                        47-bit X280 *physical* address presented to the X280
//                        front port (coherent with the X280 caches). Reaches
//                        L3 LIM (0x0800_0000), the zero device (0x0A00_0000),
//                        GDDR uncached (0x3000_0000) or cached (0x4000_3000_0000)
//                        and anything else in the X280 physical map.
//                        Granularity: 1 GiB per TLB entry.
//
// Both TLB tables reset to identity (entry i == i), so without any
// programming master 0 sees the low 4 GiB of GDDR and master 1 sees the low
// 4 GiB of X280 physical space (LIM, zero device, first 3.25 GiB of GDDR).
//
// =============================================================================
// Where things are on the NOC
// =============================================================================
//
// Everything below is addressed at the L2CPU tile's NOC coordinates.
//
//   0xFFFF_F7FE_FFF0_0000   external peripherals (X280 phys 0x2000_0000),
//                           reachable even while the X280 harts are in reset
//   0xFFFF_F7FE_FFF1_0000   L2CPU register block (reset vectors, scratch,
//                           OUTBOUND_DMA_TLB_n at +0x200, INBOUND_DMA_TLB_n
//                           at +0x240, DMA_CTRL at +0x300)
//   0x0000_0000_0000_0000.. passthrough to the X280 physical address space
//                           (L3 LIM at 0x0800_0000, GDDR at 0x3000_0000, ...)
//   0x0000_0000_2008_0000   DW_ahb_dmac registers, via that passthrough. (The
//                           bypass alias 0xFFFF_F7FE_FFF8_0000 listed in the
//                           RTL register headers does not decode on silicon.)
//
// =============================================================================

#pragma once

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <dirent.h>
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

namespace l2dma {

// =============================================================================
// Inlined tt-kmd ABI (subset)
// =============================================================================

namespace kmd {

constexpr unsigned IOCTL_MAGIC = 0xFA;
#define L2DMA_TT_IO(nr) _IO(::l2dma::kmd::IOCTL_MAGIC, nr)
constexpr unsigned long IOCTL_GET_DEVICE_INFO = L2DMA_TT_IO(0);
constexpr unsigned long IOCTL_ALLOCATE_TLB    = L2DMA_TT_IO(11);
constexpr unsigned long IOCTL_FREE_TLB        = L2DMA_TT_IO(12);
constexpr unsigned long IOCTL_CONFIGURE_TLB   = L2DMA_TT_IO(13);
#undef L2DMA_TT_IO

constexpr uint16_t BLACKHOLE_PCI_DEVICE_ID = 0xb140;

struct get_device_info_in  { __u32 output_size_bytes; };
struct get_device_info_out {
    __u32 output_size_bytes;
    __u16 vendor_id, device_id, subsystem_vendor_id, subsystem_id;
    __u16 bus_dev_fn;   // [0:2] function, [3:7] device, [8:15] bus
    __u16 max_dma_buf_size_log2;
    __u16 pci_domain;
    __u16 reserved;
};
struct get_device_info { get_device_info_in in; get_device_info_out out; };

struct allocate_tlb_in  { __u64 size; __u64 reserved; };
struct allocate_tlb_out { __u32 id; __u32 reserved0; __u64 mmap_offset_uc; __u64 mmap_offset_wc; __u64 reserved1; };
struct allocate_tlb     { allocate_tlb_in in; allocate_tlb_out out; };

struct free_tlb_in  { __u32 id; };
struct free_tlb_out {};
struct free_tlb     { free_tlb_in in; free_tlb_out out; };

struct noc_tlb_config {
    __u64 addr;
    __u16 x_end, y_end, x_start, y_start;
    __u8  noc, mcast, ordering, linked, static_vc;
    __u8  reserved0[3];
    __u32 reserved1[2];
};
struct configure_tlb_in  { __u32 id; __u32 reserved; noc_tlb_config config; };
struct configure_tlb_out { __u64 reserved; };
struct configure_tlb     { configure_tlb_in in; configure_tlb_out out; };

} // namespace kmd

// =============================================================================
// Device
// =============================================================================

class Device {
public:
    explicit Device(const std::string& path) : path_(path) {
        fd_ = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
        if (fd_ < 0) throw std::system_error(errno, std::generic_category(), "open " + path);
        kmd::get_device_info info = {};
        info.in.output_size_bytes = sizeof(info.out);
        if (ioctl(fd_, kmd::IOCTL_GET_DEVICE_INFO, &info) != 0) {
            int e = errno; ::close(fd_);
            throw std::system_error(e, std::generic_category(), "GET_DEVICE_INFO " + path);
        }
        info_ = info.out;
    }
    ~Device() { if (fd_ >= 0) ::close(fd_); }
    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    int fd() const { return fd_; }
    const std::string& path() const { return path_; }
    bool is_blackhole() const { return info_.device_id == kmd::BLACKHOLE_PCI_DEVICE_ID; }
    std::string bdf() const {
        char buf[32];
        snprintf(buf, sizeof(buf), "%04x:%02x:%02x.%x", info_.pci_domain,
                 (info_.bus_dev_fn >> 8) & 0xFF, (info_.bus_dev_fn >> 3) & 0x1F, info_.bus_dev_fn & 0x7);
        return buf;
    }

    // All /dev/tenstorrent/<N> character devices, numerically sorted.
    static std::vector<std::string> enumerate() {
        std::vector<std::pair<int, std::string>> found;
        if (DIR* d = opendir("/dev/tenstorrent")) {
            while (struct dirent* ent = readdir(d)) {
                const char* n = ent->d_name;
                if (!*n || strspn(n, "0123456789") != strlen(n)) continue;
                std::string full = std::string("/dev/tenstorrent/") + n;
                struct stat st;
                if (stat(full.c_str(), &st) == 0 && S_ISCHR(st.st_mode)) found.emplace_back(atoi(n), full);
            }
            closedir(d);
        }
        std::sort(found.begin(), found.end());
        std::vector<std::string> out;
        for (auto& f : found) out.push_back(f.second);
        return out;
    }

private:
    std::string path_;
    int fd_ = -1;
    kmd::get_device_info_out info_ = {};
};

// =============================================================================
// TlbWindow: one 2 MiB PCIe->NOC window, remapped lazily
// =============================================================================

class TlbWindow {
public:
    static constexpr uint64_t SIZE = 1ULL << 21;

    TlbWindow(int fd, bool write_combined) : fd_(fd) {
        kmd::allocate_tlb a = {};
        a.in.size = SIZE;
        if (ioctl(fd_, kmd::IOCTL_ALLOCATE_TLB, &a) != 0)
            throw std::system_error(errno, std::generic_category(), "ALLOCATE_TLB");
        id_ = a.out.id;
        off_t off = write_combined ? (off_t)a.out.mmap_offset_wc : (off_t)a.out.mmap_offset_uc;
        base_ = mmap(nullptr, SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, off);
        if (base_ == MAP_FAILED) {
            int e = errno;
            kmd::free_tlb f = {}; f.in.id = id_; ioctl(fd_, kmd::IOCTL_FREE_TLB, &f);
            throw std::system_error(e, std::generic_category(), "mmap TLB");
        }
    }
    ~TlbWindow() {
        if (base_ != MAP_FAILED) munmap(base_, SIZE);
        kmd::free_tlb f = {}; f.in.id = id_; ioctl(fd_, kmd::IOCTL_FREE_TLB, &f);
    }
    TlbWindow(const TlbWindow&) = delete;
    TlbWindow& operator=(const TlbWindow&) = delete;

    // Returns a pointer to (x, y, addr); remaps the window if needed.
    volatile uint8_t* ptr(uint16_t x, uint16_t y, uint64_t addr) {
        uint64_t aligned = addr & ~(SIZE - 1);
        if (!mapped_ || x != cur_x_ || y != cur_y_ || aligned != cur_base_) {
            kmd::configure_tlb c = {};
            c.in.id = id_;
            c.in.config.addr = aligned;
            c.in.config.x_end = x;
            c.in.config.y_end = y;
            if (ioctl(fd_, kmd::IOCTL_CONFIGURE_TLB, &c) != 0)
                throw std::system_error(errno, std::generic_category(), "CONFIGURE_TLB");
            mapped_ = true; cur_x_ = x; cur_y_ = y; cur_base_ = aligned;
        }
        return (volatile uint8_t*)base_ + (addr & (SIZE - 1));
    }
    // Bytes left in the window from addr.
    static size_t remaining(uint64_t addr) { return SIZE - (addr & (SIZE - 1)); }

private:
    int fd_;
    uint32_t id_ = 0;
    void* base_ = MAP_FAILED;
    bool mapped_ = false;
    uint16_t cur_x_ = 0, cur_y_ = 0;
    uint64_t cur_base_ = 0;
};

// =============================================================================
// Noc: typed accessors on top of a UC window (registers) and WC window (bulk)
// =============================================================================

class Noc {
public:
    explicit Noc(Device& dev) : uc_(dev.fd(), false), wc_(dev.fd(), true) {}

    uint32_t read32(uint16_t x, uint16_t y, uint64_t a) {
        check_align(a, 4);
        return *(volatile uint32_t*)uc_.ptr(x, y, a);
    }
    void write32(uint16_t x, uint16_t y, uint64_t a, uint32_t v) {
        check_align(a, 4);
        *(volatile uint32_t*)uc_.ptr(x, y, a) = v;
    }
    // Single 8-byte PCIe transaction (the DMAC registers are 64-bit wide).
    uint64_t read64(uint16_t x, uint16_t y, uint64_t a) {
        check_align(a, 8);
        return *(volatile uint64_t*)uc_.ptr(x, y, a);
    }
    void write64(uint16_t x, uint16_t y, uint64_t a, uint64_t v) {
        check_align(a, 8);
        *(volatile uint64_t*)uc_.ptr(x, y, a) = v;
    }

    // Bulk copy host -> device using 64-bit stores through the WC window.
    void write(uint16_t x, uint16_t y, uint64_t a, const void* src, size_t len) {
        check_align(a, 8); check_align(len, 8);
        const uint8_t* s = (const uint8_t*)src;
        while (len) {
            size_t chunk = std::min(len, TlbWindow::remaining(a));
            volatile uint64_t* d = (volatile uint64_t*)wc_.ptr(x, y, a);
            for (size_t i = 0; i < chunk / 8; i++) { uint64_t v; memcpy(&v, s + 8 * i, 8); d[i] = v; }
            s += chunk; a += chunk; len -= chunk;
        }
        // Flush write-combining buffers.
        __sync_synchronize();
    }
    // Bulk copy device -> host using 64-bit loads through the UC window.
    void read(uint16_t x, uint16_t y, uint64_t a, void* dst, size_t len) {
        check_align(a, 8); check_align(len, 8);
        uint8_t* d = (uint8_t*)dst;
        while (len) {
            size_t chunk = std::min(len, TlbWindow::remaining(a));
            volatile uint64_t* s = (volatile uint64_t*)uc_.ptr(x, y, a);
            for (size_t i = 0; i < chunk / 8; i++) { uint64_t v = s[i]; memcpy(d + 8 * i, &v, 8); }
            d += chunk; a += chunk; len -= chunk;
        }
    }
    void fill64(uint16_t x, uint16_t y, uint64_t a, uint64_t v, size_t len) {
        check_align(a, 8); check_align(len, 8);
        while (len) {
            size_t chunk = std::min(len, TlbWindow::remaining(a));
            volatile uint64_t* d = (volatile uint64_t*)wc_.ptr(x, y, a);
            for (size_t i = 0; i < chunk / 8; i++) d[i] = v;
            a += chunk; len -= chunk;
        }
        __sync_synchronize();
    }

private:
    static void check_align(uint64_t v, unsigned n) {
        if (v & (n - 1)) throw std::invalid_argument("misaligned NOC access");
    }
    TlbWindow uc_, wc_;
};

// =============================================================================
// L2CPU tile geometry and address constants
// =============================================================================

struct Tile { uint16_t x, y; };

// NOC0 coordinates of the four L2CPU tiles on Blackhole.
constexpr Tile L2CPU_TILE[4] = { {8, 3}, {8, 9}, {8, 5}, {8, 7} };

// NOC-side addresses on an L2CPU tile.
constexpr uint64_t PERIPH_BASE    = 0xFFFF'F7FE'FFF0'0000ULL;  // X280 phys 0x2000_0000
constexpr uint64_t L2CPU_REG_BASE = PERIPH_BASE + 0x1'0000;    // L2CPU register block
// The DW_ahb_dmac is NOT reachable through the 0xFFFF_F7FE_FFF8_0000 "bypass"
// address that the generated register headers advertise (reads return all
// ones). It is reachable through the passthrough path at its X280 physical
// address, and that works even while the X280 harts are held in reset.
constexpr uint64_t DMAC_BASE      = 0x0000'2008'0000ULL;        // DW_ahb_dmac (4 KiB)

// X280 physical addresses (reachable from the NOC by passthrough and from
// DMA master 1 through the inbound TLBs).
constexpr uint64_t X280_LIM_BASE         = 0x0000'0800'0000ULL;   // L3 LIM scratchpad, up to 2 MiB
constexpr uint64_t X280_LIM_SIZE         = 1920 * 1024;           // 15 ways x 128 KiB; way 0 is cache
constexpr uint64_t X280_ZERO_DEV_BASE    = 0x0000'0A00'0000ULL;   // 2 MiB cached zero device
constexpr uint64_t X280_GDDR_UNCACHED    = 0x0000'3000'0000ULL;   // local GDDR, 4 GiB, uncached
constexpr uint64_t X280_GDDR_CACHED      = 0x4000'3000'0000ULL;   // local GDDR, 4 GiB, through L3
constexpr uint64_t X280_NOC_SMALL_WINDOW = 0x0004'3000'0000ULL;   // 224 x 2 MiB windows, uncached
constexpr uint64_t X280_GDDR_SIZE        = 4ULL << 30;

// DDR/NOC crossbar addresses (what DMA master 0 sees after the outbound TLB).
// These are X280 system-port offsets, i.e. X280 physical minus 0x3000_0000.
constexpr uint64_t XBAR_GDDR_BASE         = 0x0000'0000'0000ULL;
constexpr uint64_t XBAR_NOC_SMALL_WINDOWS = 0x0004'0000'0000ULL;   // + n * 2 MiB
constexpr uint64_t XBAR_NOC_LARGE_WINDOWS = 0x0804'0000'0000ULL;   // + n * 128 GiB

// L2CPU register block offsets (from L2CPU_REG_BASE), 32-bit registers.
namespace l2reg {
constexpr uint32_t RESET_VECTOR_CORE0   = 0x000;   // 8 bytes per core
constexpr uint32_t SCRATCH0             = 0x100;   // 16 x 32-bit
constexpr uint32_t OUTBOUND_DMA_TLB0    = 0x200;   // 16 x 32-bit, 20-bit field
constexpr uint32_t INBOUND_DMA_TLB0     = 0x240;   // 4 x 32-bit, 17-bit field
constexpr uint32_t DMA_CTRL             = 0x300;
constexpr uint32_t X280_MANAGEMENT      = 0x400;
constexpr int NUM_OUTBOUND_TLB = 16;
constexpr int NUM_INBOUND_TLB  = 4;
constexpr uint32_t OUTBOUND_TLB_MASK = (1u << 20) - 1;
constexpr uint32_t INBOUND_TLB_MASK  = (1u << 17) - 1;
constexpr unsigned OUTBOUND_SHIFT = 28;   // 256 MiB granule
constexpr unsigned INBOUND_SHIFT  = 30;   // 1 GiB granule

// DMA_CTRL fields
constexpr uint32_t DMA_CTRL_STATIC_CG       = 1u << 0;
constexpr uint32_t DMA_CTRL_DYNAMIC_CG      = 1u << 1;
constexpr uint32_t DMA_CTRL_NOC2DMA_AW_RSZ  = 1u << 2;
constexpr uint32_t DMA_CTRL_NOC2DMA_AR_RSZ  = 1u << 3;
constexpr uint32_t DMA_CTRL_NOC2DMA_CACHE_SHIFT = 4;   // 4 bits
constexpr uint32_t DMA_CTRL_DMA2L2_AW_RSZ   = 1u << 8;
constexpr uint32_t DMA_CTRL_DMA2L2_AR_RSZ   = 1u << 9;
constexpr uint32_t DMA_CTRL_DMA2L2_CACHE_SHIFT  = 10;  // 4 bits
constexpr uint32_t DMA_CTRL_INT_MASK_SHIFT      = 16;  // 5 bits: tfr, block, srctran, dsttran, err
constexpr uint32_t DMA_CTRL_RESET_VALUE         = 0x000E0B2E;
} // namespace l2reg

// DW_ahb_dmac register offsets (from DMAC_BASE). All registers are 64 bits.
namespace dmac {
constexpr uint32_t CH_STRIDE = 0x58;
constexpr int NUM_CHANNELS = 2;
// per channel, add ch * CH_STRIDE
constexpr uint32_t SAR = 0x00, DAR = 0x08, LLP = 0x10, CTL = 0x18;
constexpr uint32_t SSTAT = 0x20, DSTAT = 0x28, SSTATAR = 0x30, DSTATAR = 0x38;
constexpr uint32_t CFG = 0x40, SGR = 0x48, DSR = 0x50;
// interrupt block
constexpr uint32_t RAW_TFR = 0x2C0, RAW_BLOCK = 0x2C8, RAW_SRCTRAN = 0x2D0, RAW_DSTTRAN = 0x2D8, RAW_ERR = 0x2E0;
constexpr uint32_t STATUS_TFR = 0x2E8, STATUS_BLOCK = 0x2F0, STATUS_SRCTRAN = 0x2F8, STATUS_DSTTRAN = 0x300, STATUS_ERR = 0x308;
constexpr uint32_t MASK_TFR = 0x310, MASK_BLOCK = 0x318, MASK_SRCTRAN = 0x320, MASK_DSTTRAN = 0x328, MASK_ERR = 0x330;
constexpr uint32_t CLEAR_TFR = 0x338, CLEAR_BLOCK = 0x340, CLEAR_SRCTRAN = 0x348, CLEAR_DSTTRAN = 0x350, CLEAR_ERR = 0x358;
constexpr uint32_t STATUS_INT = 0x360;
// software handshake block (unused for memory-to-memory)
constexpr uint32_t REQ_SRC = 0x368, REQ_DST = 0x370, SGL_REQ_SRC = 0x378, SGL_REQ_DST = 0x380, LST_SRC = 0x388, LST_DST = 0x390;
// misc
constexpr uint32_t DMA_CFG = 0x398, CH_EN = 0x3A0, DMA_ID = 0x3A8, DMA_TEST = 0x3B0;
constexpr uint32_t COMP_PARAMS_BASE = 0x3C8;   // 32-bit words up to 0x3F8
constexpr uint32_t COMP_ID = 0x3F8;            // [31:0] type, [63:32] version

// Mask/ChEn/Req registers: value in bits [1:0], write-enable in bits [9:8].
constexpr unsigned WE_SHIFT = 8;
inline uint64_t we(unsigned ch, bool set) { return (1ULL << (WE_SHIFT + ch)) | ((uint64_t)set << ch); }

// CTL fields
constexpr unsigned CTL_INT_EN        = 0;
constexpr unsigned CTL_DST_TR_WIDTH  = 1;   // 3 bits: 0=1B 1=2B 2=4B 3=8B 4=16B 5=32B
constexpr unsigned CTL_SRC_TR_WIDTH  = 4;   // 3 bits
constexpr unsigned CTL_DINC          = 7;   // 2 bits: 0=inc 1=dec 2=fixed
constexpr unsigned CTL_SINC          = 9;   // 2 bits
constexpr unsigned CTL_DEST_MSIZE    = 11;  // 3 bits: 0=1 1=4 2=8 3=16 4=32 5=64 6=128 7=256 items
constexpr unsigned CTL_SRC_MSIZE     = 14;  // 3 bits
constexpr unsigned CTL_SRC_GATHER_EN = 17;
constexpr unsigned CTL_DST_SCATTER_EN= 18;
constexpr unsigned CTL_TT_FC         = 20;  // 3 bits: 0 = mem->mem, DMA is flow controller
constexpr unsigned CTL_DMS           = 23;  // 2 bits: destination master
constexpr unsigned CTL_SMS           = 25;  // 2 bits: source master
constexpr unsigned CTL_LLP_DST_EN    = 27;
constexpr unsigned CTL_LLP_SRC_EN    = 28;
constexpr unsigned CTL_BLOCK_TS      = 32;  // 12 bits, in units of SRC_TR_WIDTH
constexpr unsigned CTL_DONE          = 44;
constexpr uint32_t MAX_BLOCK_TS      = 4095;

// CFG fields
constexpr unsigned CFG_CH_PRIOR   = 5;   // 3 bits
constexpr unsigned CFG_CH_SUSP    = 8;
constexpr unsigned CFG_FIFO_EMPTY = 9;   // read-only
constexpr unsigned CFG_HS_SEL_DST = 10;  // 1 = software (no hardware handshake)
constexpr unsigned CFG_HS_SEL_SRC = 11;
constexpr unsigned CFG_MAX_ABRST  = 20;  // 10 bits
constexpr unsigned CFG_RELOAD_SRC = 30;
constexpr unsigned CFG_RELOAD_DST = 31;
constexpr unsigned CFG_FCMODE     = 32;
constexpr unsigned CFG_FIFO_MODE  = 33;
constexpr unsigned CFG_PROTCTL    = 34;  // 3 bits
constexpr unsigned CFG_DS_UPD_EN  = 37;
constexpr unsigned CFG_SS_UPD_EN  = 38;
} // namespace dmac

// =============================================================================
// X280 NOC TLB windows (the tile's own windows onto the NOC)
// =============================================================================
//
// Three unrelated "TLB" layers exist on this path, and this library names
// them separately:
//   1. PCIe -> NOC windows, owned by tt-kmd (TlbWindow above). How the host
//      reaches any tile.
//   2. L2CPU DMA address-expansion TLBs (set_outbound_tlb / set_inbound_tlb).
//      How the DMAC's 32-bit AHB addresses become crossbar / X280 addresses.
//   3. X280 -> NOC windows, configured here. 224 small (2 MiB) and 32 large
//      (128 GiB) windows that the X280 system port, and thus DMA master 0
//      via XBAR_NOC_SMALL_WINDOWS / XBAR_NOC_LARGE_WINDOWS, use to reach
//      other tiles (Tensix L1, other DRAM, the PCIe tile, ...).
//
// Configuration lives in the external-peripherals block: 16 bytes per small
// window at PERIPH_BASE + 16*n, 12 bytes per large window at
// PERIPH_BASE + 0xE00 + 12*n. See tt-isa-documentation TLBWindows.md. The
// local_offset word holds the target NOC address in units of the window size
// (address >> 21 for small, >> 37 for large windows). Coordinates are in the
// same translated space the host's PCIe TLBs use; the L2CPU NIUs have the
// same translation tables.

namespace nocwin {
constexpr unsigned NUM_SMALL  = 224;
constexpr unsigned NUM_LARGE  = 32;
constexpr uint64_t SMALL_SIZE = 2ULL << 20;
constexpr uint64_t LARGE_SIZE = 128ULL << 30;
constexpr uint64_t SMALL_CFG  = PERIPH_BASE + 0x000;
constexpr uint64_t LARGE_CFG  = PERIPH_BASE + 0xE00;
// noc_properties_lo
inline uint32_t props_lo(uint16_t x_end, uint16_t y_end, uint16_t x_start = 0, uint16_t y_start = 0,
                         bool mcast = false, unsigned ordering = 0, bool linked = false, bool static_vc = false, unsigned noc_sel = 0) {
    return (x_end & 0x3F) | ((y_end & 0x3F) << 6) | ((x_start & 0x3F) << 12) | ((y_start & 0x3F) << 18) |
           ((uint32_t)mcast << 24) | ((ordering & 3) << 25) | ((uint32_t)linked << 27) | ((uint32_t)static_vc << 28) |
           ((noc_sel & 1) << 31);
}
} // namespace nocwin

// =============================================================================
// Transfer description
// =============================================================================

// Which AHB master of the DMAC an address refers to.
enum class Master : uint8_t {
    Extern = 0,   // master 0: outbound TLBs -> DDR/NOC crossbar (GDDR, NOC windows)
    L2     = 1,   // master 1: inbound TLBs  -> X280 front port (LIM, GDDR, anything X280 sees)
};

// A 32-bit DMAC-side address on a given master.
struct DmaAddr {
    Master   master;
    uint32_t addr;
};

inline const char* master_name(Master m) { return m == Master::Extern ? "EXTERN" : "L2"; }

struct XferOptions {
    unsigned width_bytes = 32;     // 1,2,4,8,16,32 - source and destination transfer width
    unsigned burst_items = 256;    // 1,4,8,16,32,64,128,256 - MSIZE for both sides
    double   timeout_s   = 2.0;
    bool     verbose     = false;
    // Where to build the linked list for transfers bigger than one block.
    // Must be reachable from `lli_master` at `lli_dma_addr`, and from the host
    // through NOC passthrough at `lli_x280_phys`. Default: last 64 KiB of the
    // L3 LIM (which must have been ECC-scrubbed, see scrub_l3_lim()).
    Master   lli_master   = Master::L2;
    uint32_t lli_dma_addr = 0x0800'0000 + 0x1D'0000;   // L2 master, identity TLB 0
    uint64_t lli_x280_phys= X280_LIM_BASE + 0x1D'0000;
};

struct XferResult {
    bool     ok = false;
    bool     error_irq = false;
    bool     timed_out = false;
    uint32_t blocks = 0;
    double   seconds = 0.0;
    uint64_t bytes = 0;
    double   mib_per_s() const { return seconds > 0 ? (double)bytes / seconds / (1024.0 * 1024.0) : 0.0; }
};

// =============================================================================
// Dmac: one L2CPU tile's DMA engine
// =============================================================================

class Dmac {
public:
    Dmac(Noc& noc, int tile_index) : noc_(noc), idx_(tile_index), t_(L2CPU_TILE[tile_index]) {
        if (tile_index < 0 || tile_index > 3) throw std::invalid_argument("L2CPU index must be 0..3");
    }

    int  index() const { return idx_; }
    Tile tile()  const { return t_; }

    // ---- raw register access ------------------------------------------------
    uint64_t rd(uint32_t off)              { return noc_.read64(t_.x, t_.y, DMAC_BASE + off); }
    void     wr(uint32_t off, uint64_t v)  { noc_.write64(t_.x, t_.y, DMAC_BASE + off, v); }
    uint64_t rd_ch(unsigned ch, uint32_t off)             { return rd(ch * dmac::CH_STRIDE + off); }
    void     wr_ch(unsigned ch, uint32_t off, uint64_t v) { wr(ch * dmac::CH_STRIDE + off, v); }
    uint32_t rd_l2reg(uint32_t off)             { return noc_.read32(t_.x, t_.y, L2CPU_REG_BASE + off); }
    void     wr_l2reg(uint32_t off, uint32_t v) { noc_.write32(t_.x, t_.y, L2CPU_REG_BASE + off, v); }

    // X280 physical address space of this tile, through NOC passthrough.
    void     phys_write(uint64_t phys, const void* src, size_t len) { noc_.write(t_.x, t_.y, phys, src, len); }
    void     phys_read (uint64_t phys, void* dst, size_t len)       { noc_.read (t_.x, t_.y, phys, dst, len); }
    void     phys_fill64(uint64_t phys, uint64_t v, size_t len)     { noc_.fill64(t_.x, t_.y, phys, v, len); }
    uint64_t phys_read64(uint64_t phys)                              { return noc_.read64(t_.x, t_.y, phys); }
    void     phys_write64(uint64_t phys, uint64_t v)                 { noc_.write64(t_.x, t_.y, phys, v); }

    // ---- DMA TLBs -----------------------------------------------------------
    // Outbound (master 0): DMAC address bits [31:28] == slot maps to
    // crossbar address xbar_base (256 MiB aligned).
    void set_outbound_tlb(unsigned slot, uint64_t xbar_base) {
        if (slot >= l2reg::NUM_OUTBOUND_TLB) throw std::invalid_argument("outbound slot");
        if (xbar_base & ((1ULL << l2reg::OUTBOUND_SHIFT) - 1)) throw std::invalid_argument("outbound base must be 256 MiB aligned");
        uint64_t v = xbar_base >> l2reg::OUTBOUND_SHIFT;
        if (v > l2reg::OUTBOUND_TLB_MASK) throw std::invalid_argument("outbound base out of range");
        wr_l2reg(l2reg::OUTBOUND_DMA_TLB0 + 4 * slot, (uint32_t)v);
    }
    uint64_t get_outbound_tlb(unsigned slot) {
        return (uint64_t)(rd_l2reg(l2reg::OUTBOUND_DMA_TLB0 + 4 * slot) & l2reg::OUTBOUND_TLB_MASK) << l2reg::OUTBOUND_SHIFT;
    }
    // Inbound (master 1): DMAC address bits [31:30] == slot maps to X280
    // physical address phys_base (1 GiB aligned).
    void set_inbound_tlb(unsigned slot, uint64_t phys_base) {
        if (slot >= l2reg::NUM_INBOUND_TLB) throw std::invalid_argument("inbound slot");
        if (phys_base & ((1ULL << l2reg::INBOUND_SHIFT) - 1)) throw std::invalid_argument("inbound base must be 1 GiB aligned");
        uint64_t v = phys_base >> l2reg::INBOUND_SHIFT;
        if (v > l2reg::INBOUND_TLB_MASK) throw std::invalid_argument("inbound base out of range");
        wr_l2reg(l2reg::INBOUND_DMA_TLB0 + 4 * slot, (uint32_t)v);
    }
    uint64_t get_inbound_tlb(unsigned slot) {
        return (uint64_t)(rd_l2reg(l2reg::INBOUND_DMA_TLB0 + 4 * slot) & l2reg::INBOUND_TLB_MASK) << l2reg::INBOUND_SHIFT;
    }
    void reset_tlbs_to_identity() {
        for (unsigned i = 0; i < l2reg::NUM_OUTBOUND_TLB; i++) wr_l2reg(l2reg::OUTBOUND_DMA_TLB0 + 4 * i, i);
        for (unsigned i = 0; i < l2reg::NUM_INBOUND_TLB; i++)  wr_l2reg(l2reg::INBOUND_DMA_TLB0 + 4 * i, i);
    }

    // Convenience address builders.
    // Master 0 address for crossbar address `xbar` via outbound slot `slot`
    // (slot must already be mapped so that xbar lies inside its 256 MiB).
    DmaAddr extern_addr(unsigned slot, uint64_t xbar) {
        uint64_t base = get_outbound_tlb(slot);
        if (xbar < base || xbar - base >= (1ULL << l2reg::OUTBOUND_SHIFT)) throw std::invalid_argument("xbar address not inside outbound slot");
        return { Master::Extern, (uint32_t)((slot << l2reg::OUTBOUND_SHIFT) | (xbar - base)) };
    }
    // Master 1 address for X280 physical address `phys` via inbound slot.
    DmaAddr l2_addr(unsigned slot, uint64_t phys) {
        uint64_t base = get_inbound_tlb(slot);
        if (phys < base || phys - base >= (1ULL << l2reg::INBOUND_SHIFT)) throw std::invalid_argument("phys address not inside inbound slot");
        return { Master::L2, (uint32_t)((slot << l2reg::INBOUND_SHIFT) | (phys - base)) };
    }

    // ---- X280 NOC TLB windows (layer 3, see nocwin) -------------------------
    // Point small window n at (x, y, noc_addr). noc_addr must be 2 MiB
    // aligned. Returns the crossbar address of the window, which is what an
    // outbound DMA TLB entry must cover.
    // `ordering`: 0 default, 1 strict AXI (needed for reads from DRAM tiles,
    // whose controllers may return responses out of order), 2 posted writes, 3 counted writes.
    uint64_t set_noc_small_window(unsigned n, uint16_t x, uint16_t y, uint64_t noc_addr, unsigned noc_sel = 0, unsigned ordering = 0) {
        if (n >= nocwin::NUM_SMALL) throw std::invalid_argument("small window index");
        if (noc_addr & (nocwin::SMALL_SIZE - 1)) throw std::invalid_argument("small window address must be 2 MiB aligned");
        uint64_t cfg = nocwin::SMALL_CFG + 16ULL * n;
        noc_.write64(t_.x, t_.y, cfg, noc_addr >> 21);   // local_offset is in units of 2 MiB
        noc_.write32(t_.x, t_.y, cfg + 8, nocwin::props_lo(x, y, 0, 0, false, ordering, false, false, noc_sel));
        noc_.write32(t_.x, t_.y, cfg + 12, 0);
        return XBAR_NOC_SMALL_WINDOWS + nocwin::SMALL_SIZE * n;
    }
    // Point large window n at (x, y, noc_addr). noc_addr must be 128 GiB aligned.
    uint64_t set_noc_large_window(unsigned n, uint16_t x, uint16_t y, uint64_t noc_addr, unsigned noc_sel = 0, unsigned ordering = 0) {
        if (n >= nocwin::NUM_LARGE) throw std::invalid_argument("large window index");
        if (noc_addr & (nocwin::LARGE_SIZE - 1)) throw std::invalid_argument("large window address must be 128 GiB aligned");
        uint64_t cfg = nocwin::LARGE_CFG + 12ULL * n;
        noc_.write32(t_.x, t_.y, cfg, (uint32_t)(noc_addr >> 37));
        noc_.write32(t_.x, t_.y, cfg + 4, nocwin::props_lo(x, y, 0, 0, false, ordering, false, false, noc_sel));
        noc_.write32(t_.x, t_.y, cfg + 8, 0);
        return XBAR_NOC_LARGE_WINDOWS + nocwin::LARGE_SIZE * n;
    }
    // Make outbound DMA slot `slot` cover small window n and return the
    // master-0 address of byte `offset` inside that window.
    DmaAddr extern_noc_small(unsigned slot, unsigned n, uint64_t offset) {
        uint64_t win = XBAR_NOC_SMALL_WINDOWS + nocwin::SMALL_SIZE * n;
        set_outbound_tlb(slot, win & ~((1ULL << l2reg::OUTBOUND_SHIFT) - 1));
        return extern_addr(slot, win + offset);
    }

    // ---- controller state ---------------------------------------------------
    void enable()  { wr(dmac::DMA_CFG, 1); }
    void disable() { wr(dmac::DMA_CFG, 0); }
    bool enabled() { return rd(dmac::DMA_CFG) & 1; }
    bool channel_busy(unsigned ch) { return (rd(dmac::CH_EN) >> ch) & 1; }
    void channel_enable(unsigned ch)  { wr(dmac::CH_EN, dmac::we(ch, true)); }
    void channel_disable(unsigned ch) { wr(dmac::CH_EN, dmac::we(ch, false)); }
    void clear_all_irqs(unsigned ch) {
        uint64_t b = 1ULL << ch;
        wr(dmac::CLEAR_TFR, b); wr(dmac::CLEAR_BLOCK, b); wr(dmac::CLEAR_SRCTRAN, b);
        wr(dmac::CLEAR_DSTTRAN, b); wr(dmac::CLEAR_ERR, b);
    }
    void unmask_irqs(unsigned ch) {
        uint64_t v = dmac::we(ch, true);
        wr(dmac::MASK_TFR, v); wr(dmac::MASK_BLOCK, v); wr(dmac::MASK_ERR, v);
        wr(dmac::MASK_SRCTRAN, dmac::we(ch, false)); wr(dmac::MASK_DSTTRAN, dmac::we(ch, false));
    }

    // Wait until the channel is idle; returns false on timeout.
    bool wait_idle(unsigned ch, double timeout_s) {
        auto t0 = std::chrono::steady_clock::now();
        while (channel_busy(ch)) {
            double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (el > timeout_s) return false;
        }
        return true;
    }

    // Abort a channel: suspend, wait for FIFO empty, disable.
    void abort(unsigned ch) {
        uint64_t cfg = rd_ch(ch, dmac::CFG);
        wr_ch(ch, dmac::CFG, cfg | (1ULL << dmac::CFG_CH_SUSP));
        auto t0 = std::chrono::steady_clock::now();
        while (!((rd_ch(ch, dmac::CFG) >> dmac::CFG_FIFO_EMPTY) & 1)) {
            if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > 0.5) break;
        }
        channel_disable(ch);
        wr_ch(ch, dmac::CFG, cfg & ~(1ULL << dmac::CFG_CH_SUSP));
    }

    // ---- building CTL values ------------------------------------------------
    static unsigned width_code(unsigned bytes) {
        switch (bytes) { case 1: return 0; case 2: return 1; case 4: return 2; case 8: return 3; case 16: return 4; case 32: return 5; }
        throw std::invalid_argument("width must be 1,2,4,8,16 or 32 bytes");
    }
    static unsigned msize_code(unsigned items) {
        switch (items) { case 1: return 0; case 4: return 1; case 8: return 2; case 16: return 3; case 32: return 4; case 64: return 5; case 128: return 6; case 256: return 7; }
        throw std::invalid_argument("burst must be 1,4,8,16,32,64,128 or 256 items");
    }
    static uint64_t make_ctl(Master src, Master dst, unsigned width_bytes, unsigned burst_items,
                             uint32_t block_ts, bool llp_chain, bool int_en = true) {
        uint64_t w = width_code(width_bytes), m = msize_code(burst_items);
        uint64_t ctl = 0;
        ctl |= (uint64_t)int_en << dmac::CTL_INT_EN;
        ctl |= w << dmac::CTL_DST_TR_WIDTH;
        ctl |= w << dmac::CTL_SRC_TR_WIDTH;
        ctl |= 0ULL << dmac::CTL_DINC;   // increment
        ctl |= 0ULL << dmac::CTL_SINC;   // increment
        ctl |= m << dmac::CTL_DEST_MSIZE;
        ctl |= m << dmac::CTL_SRC_MSIZE;
        ctl |= 0ULL << dmac::CTL_TT_FC;  // memory to memory, DMAC is flow controller
        ctl |= (uint64_t)dst << dmac::CTL_DMS;
        ctl |= (uint64_t)src << dmac::CTL_SMS;
        ctl |= (uint64_t)llp_chain << dmac::CTL_LLP_DST_EN;
        ctl |= (uint64_t)llp_chain << dmac::CTL_LLP_SRC_EN;
        ctl |= (uint64_t)block_ts << dmac::CTL_BLOCK_TS;
        return ctl;
    }
    static uint64_t make_cfg(unsigned ch) {
        uint64_t cfg = 0;
        cfg |= (uint64_t)ch << dmac::CFG_CH_PRIOR;
        cfg |= 1ULL << dmac::CFG_HS_SEL_DST;   // "software" = no hardware handshake
        cfg |= 1ULL << dmac::CFG_HS_SEL_SRC;
        return cfg;
    }

    // ---- transfers ----------------------------------------------------------
    // Program and start a copy of `bytes` from src to dst on channel `ch`
    // without waiting. Uses a single block when it fits, otherwise builds a
    // linked list (see XferOptions::lli_*; one list area per channel is the
    // caller's responsibility when running both channels at once).
    void start(unsigned ch, DmaAddr src, DmaAddr dst, uint64_t bytes, const XferOptions& o = XferOptions()) {
        if (ch >= dmac::NUM_CHANNELS) throw std::invalid_argument("channel must be 0 or 1");
        if (bytes == 0 || bytes % o.width_bytes) throw std::invalid_argument("bytes must be a non-zero multiple of width");
        if ((src.addr | dst.addr) & (o.width_bytes - 1)) throw std::invalid_argument("src/dst must be aligned to width");

        const uint64_t max_block_bytes = (uint64_t)dmac::MAX_BLOCK_TS * o.width_bytes;
        const uint32_t nblocks = (uint32_t)((bytes + max_block_bytes - 1) / max_block_bytes);
        pending_[ch] = { bytes, nblocks, std::chrono::steady_clock::now() };

        if (!enabled()) enable();
        if (channel_busy(ch)) abort(ch);
        clear_all_irqs(ch);
        unmask_irqs(ch);
        wr_ch(ch, dmac::CFG, make_cfg(ch));
        wr_ch(ch, dmac::SGR, 0);
        wr_ch(ch, dmac::DSR, 0);

        if (nblocks == 1) {
            uint32_t block_ts = (uint32_t)(bytes / o.width_bytes);
            wr_ch(ch, dmac::SAR, src.addr);
            wr_ch(ch, dmac::DAR, dst.addr);
            wr_ch(ch, dmac::LLP, 0);
            wr_ch(ch, dmac::CTL, make_ctl(src.master, dst.master, o.width_bytes, o.burst_items, block_ts, false));
        } else {
            build_lli(src, dst, bytes, nblocks, max_block_bytes, o);
            // SAR/DAR/CTL are loaded from the first LLI; LLP points at it.
            wr_ch(ch, dmac::SAR, src.addr);
            wr_ch(ch, dmac::DAR, dst.addr);
            wr_ch(ch, dmac::LLP, (uint64_t)(o.lli_dma_addr & ~3u) | (uint64_t)o.lli_master);
            wr_ch(ch, dmac::CTL, make_ctl(src.master, dst.master, o.width_bytes, o.burst_items, 0, true));
        }
        if (o.verbose) {
            printf("    ch%u: %s:0x%08x -> %s:0x%08x  %llu bytes, %u block(s), width %u, burst %u\n", ch,
                   master_name(src.master), src.addr, master_name(dst.master), dst.addr,
                   (unsigned long long)bytes, nblocks, o.width_bytes, o.burst_items);
        }
        pending_[ch].t0 = std::chrono::steady_clock::now();
        channel_enable(ch);
    }

    // Wait for a transfer started with start() and report on it.
    XferResult wait(unsigned ch, double timeout_s = 2.0) {
        XferResult r;
        r.bytes = pending_[ch].bytes;
        r.blocks = pending_[ch].blocks;
        bool idle = wait_idle(ch, timeout_s);
        r.seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - pending_[ch].t0).count();
        uint64_t raw_err = rd(dmac::RAW_ERR), raw_tfr = rd(dmac::RAW_TFR);
        r.error_irq = (raw_err >> ch) & 1;
        r.timed_out = !idle;
        if (!idle) abort(ch);
        r.ok = idle && !r.error_irq && ((raw_tfr >> ch) & 1);
        clear_all_irqs(ch);
        return r;
    }

    // start() + wait(): copy `bytes` from src to dst and block until done.
    XferResult copy(unsigned ch, DmaAddr src, DmaAddr dst, uint64_t bytes, const XferOptions& o = XferOptions()) {
        start(ch, src, dst, bytes, o);
        return wait(ch, o.timeout_s);
    }

    // Dump the interesting registers.
    void dump(FILE* f = stdout) {
        fprintf(f, "  L2CPU %d (%u,%u) DMAC @ 0x%016llx\n", idx_, t_.x, t_.y, (unsigned long long)DMAC_BASE);
        fprintf(f, "    COMP_ID   %016llx  DMA_ID %016llx  DMA_CFG %016llx  CH_EN %016llx\n",
                (unsigned long long)rd(dmac::COMP_ID), (unsigned long long)rd(dmac::DMA_ID),
                (unsigned long long)rd(dmac::DMA_CFG), (unsigned long long)rd(dmac::CH_EN));
        for (unsigned ch = 0; ch < dmac::NUM_CHANNELS; ch++) {
            fprintf(f, "    ch%u SAR %08llx DAR %08llx LLP %08llx CTL %016llx CFG %016llx\n", ch,
                    (unsigned long long)rd_ch(ch, dmac::SAR), (unsigned long long)rd_ch(ch, dmac::DAR),
                    (unsigned long long)rd_ch(ch, dmac::LLP), (unsigned long long)rd_ch(ch, dmac::CTL),
                    (unsigned long long)rd_ch(ch, dmac::CFG));
        }
        fprintf(f, "    RAW  tfr %llx block %llx src %llx dst %llx err %llx   STATUS_INT %llx\n",
                (unsigned long long)rd(dmac::RAW_TFR), (unsigned long long)rd(dmac::RAW_BLOCK),
                (unsigned long long)rd(dmac::RAW_SRCTRAN), (unsigned long long)rd(dmac::RAW_DSTTRAN),
                (unsigned long long)rd(dmac::RAW_ERR), (unsigned long long)rd(dmac::STATUS_INT));
        fprintf(f, "    DMA_CTRL %08x   outbound TLB:", rd_l2reg(l2reg::DMA_CTRL));
        for (unsigned i = 0; i < l2reg::NUM_OUTBOUND_TLB; i++) fprintf(f, " %05x", rd_l2reg(l2reg::OUTBOUND_DMA_TLB0 + 4 * i));
        fprintf(f, "\n                       inbound TLB:");
        for (unsigned i = 0; i < l2reg::NUM_INBOUND_TLB; i++) fprintf(f, " %05x", rd_l2reg(l2reg::INBOUND_DMA_TLB0 + 4 * i));
        fprintf(f, "\n");
    }

private:
    // Linked list item as the DW_ahb_dmac fetches it (32-bit words).
    struct Lli {
        uint32_t sar, dar, llp, ctl_lo, ctl_hi, sstat, dstat, pad;
    };
    static_assert(sizeof(Lli) == 32, "LLI must be 32 bytes");

    void build_lli(DmaAddr src, DmaAddr dst, uint64_t bytes, uint32_t nblocks, uint64_t max_block_bytes, const XferOptions& o) {
        std::vector<Lli> list(nblocks);
        uint64_t off = 0;
        for (uint32_t i = 0; i < nblocks; i++) {
            uint64_t len = std::min(max_block_bytes, bytes - off);
            bool last = (i + 1 == nblocks);
            uint64_t ctl = make_ctl(src.master, dst.master, o.width_bytes, o.burst_items, (uint32_t)(len / o.width_bytes), !last);
            Lli& l = list[i];
            l.sar = (uint32_t)(src.addr + off);
            l.dar = (uint32_t)(dst.addr + off);
            l.llp = last ? 0 : (uint32_t)(((o.lli_dma_addr + (i + 1) * sizeof(Lli)) & ~3u) | (uint32_t)o.lli_master);
            l.ctl_lo = (uint32_t)ctl;
            l.ctl_hi = (uint32_t)(ctl >> 32);
            l.sstat = l.dstat = l.pad = 0;
            off += len;
        }
        phys_write(o.lli_x280_phys, list.data(), list.size() * sizeof(Lli));
    }

    struct Pending { uint64_t bytes = 0; uint32_t blocks = 0; std::chrono::steady_clock::time_point t0; };
    Pending pending_[dmac::NUM_CHANNELS];
    Noc& noc_;
    int  idx_;
    Tile t_;
};

// =============================================================================
// L3 cache controller, LIM ECC, and booting harts
// =============================================================================
//
// The L3 LIM is SRAM with ECC and nothing initialises the ECC bits at power
// up. A read of a never-written line returns an AXI error (and bumps the L3
// DatECCFail counter). Writes through the X280 front port - which is how
// both the host (over the NOC) and the DMA engine reach the LIM - do not fix
// this; the SiFive manual (18.1.1) says ECC SRAM must be initialised with
// 64-bit stores from the processor or an internal DMA and "initialising
// memory with ECC from an external bus is not recommended". So on a fresh
// chip the LIM is DMA-writable but not DMA-readable.
//
// The fix is to let a hart zero the LIM once: scrub_l3_lim() loads a tiny
// payload into GDDR, boots the cluster, and waits for hart 0 to report that
// it has stored zeros over the whole LIM. The ECC state survives ASIC
// resets, so this is needed once per power cycle. Note the Blackhole
// hardware bug that harts can only be released from reset once per ASIC
// reset; after scrubbing, the harts stay parked in wfi.

namespace l3 {
constexpr uint64_t BASE        = 0x0000'0201'0000ULL;   // X280 phys
constexpr uint32_t CONFIG      = 0x000;
constexpr uint32_t WAY_ENABLE  = 0x008;   // R: largest enabled way; W: may only increase
constexpr uint32_t DIR_ECC_FIX_ADDR  = 0x100, DIR_ECC_FIX_COUNT  = 0x108;
constexpr uint32_t DIR_ECC_FAIL_ADDR = 0x120, DIR_ECC_FAIL_COUNT = 0x128;
constexpr uint32_t DAT_ECC_FIX_ADDR  = 0x140, DAT_ECC_FIX_COUNT  = 0x148;
constexpr uint32_t DAT_ECC_FAIL_ADDR = 0x160, DAT_ECC_FAIL_COUNT = 0x168;
constexpr uint32_t FLUSH64     = 0x200;
constexpr uint32_t FLUSH32     = 0x240;
constexpr uint32_t WAY_MASK0   = 0x800;   // one 64-bit register per master id
} // namespace l3

// ARC-tile PLL that clocks the L2CPU tiles ("PLL4").
namespace pll {
constexpr Tile     ARC        = { 8, 0 };
constexpr uint64_t PLL4_BASE  = 0x8002'0500ULL;
constexpr uint64_t CNTL_1     = PLL4_BASE + 0x04;   // [31:16] fbdiv
constexpr uint64_t CNTL_5     = PLL4_BASE + 0x14;   // 4 x 8-bit postdiv
struct Config { uint16_t fbdiv; uint8_t postdiv[4]; };
constexpr Config MHZ_200  = { 128, { 15, 15, 15, 15 } };
constexpr Config MHZ_800  = {  64, {  1,  1,  1,  1 } };   // power-on default
constexpr Config MHZ_1750 = { 140, {  1,  1,  1,  1 } };
} // namespace pll

// ARC-tile register that releases L2CPU clusters from reset: bits [7:4],
// one per cluster, 0 -> 1 releases. Can only be done once per ASIC reset.
constexpr uint64_t ARC_L2CPU_RESET = 0x8003'0014ULL;

// Step the L2CPU PLL to `target` one divider tick at a time (from x280_tester).
inline void set_l2cpu_pll(Noc& noc, const pll::Config& target) {
    uint32_t cntl1 = noc.read32(pll::ARC.x, pll::ARC.y, pll::CNTL_1);
    uint32_t cntl5 = noc.read32(pll::ARC.x, pll::ARC.y, pll::CNTL_5);
    uint16_t fbdiv = (cntl1 >> 16) & 0xFFFF;
    uint8_t  pd[4] = { (uint8_t)cntl5, (uint8_t)(cntl5 >> 8), (uint8_t)(cntl5 >> 16), (uint8_t)(cntl5 >> 24) };
    auto write_pd = [&] {
        cntl5 = pd[0] | (pd[1] << 8) | (pd[2] << 16) | (pd[3] << 24);
        noc.write32(pll::ARC.x, pll::ARC.y, pll::CNTL_5, cntl5);
    };
    for (int i = 0; i < 4; i++) while (pd[i] < target.postdiv[i]) { pd[i]++; write_pd(); }
    while (fbdiv != target.fbdiv) {
        fbdiv += (fbdiv < target.fbdiv) ? 1 : -1;
        cntl1 = (cntl1 & 0xFFFF) | ((uint32_t)fbdiv << 16);
        noc.write32(pll::ARC.x, pll::ARC.y, pll::CNTL_1, cntl1);
    }
    for (int i = 0; i < 4; i++) while (pd[i] > target.postdiv[i]) { pd[i]--; write_pd(); }
}

// Hart 0 zeroes the L3 LIM (0x0800_0000 .. 0x081E_0000) with 64-bit
// stores, writes 0x5C7B00D0 to the 32-bit word at payload+0x3C and parks in
// wfi; harts 1..3 park immediately. Position independent, 60 bytes, so it
// fits in the tile's 64-byte general purpose scratch memory (the 16 SCRATCH
// registers at X280 phys 0x2001_0100), which needs no working DRAM or LIM.
// Source: see README.
constexpr uint32_t SCRUB_PAYLOAD[16] = {
    0xf14022f3, // csrr  t0, mhartid
    0x02029863, // bnez  t0, park
    0x00000317, // auipc t1, 0
    0x08000537, // lui   a0, 0x08000      LIM start
    0x081e05b7, // lui   a1, 0x081E0      LIM end
    0x00053023, // loop: sd zero, 0(a0)
    0x00850513, //       addi a0, a0, 8
    0xfeb56ce3, //       bltu a0, a1, loop
    0x0ff0000f, // fence
    0x5c7b03b7, // lui   t2, 0x5C7B0
    0x0d03e393, // ori   t2, t2, 0xD0     -> 0x5C7B00D0
    0x02732a23, // sw    t2, 0x34(t1)     t1 = base+8, so this is base+0x3C: the last scratch word
    0x0ff0000f, // fence
    0x10500073, // park: wfi
    0xffdff06f, //       j park
    0x00000000, // flag slot
};
constexpr uint32_t SCRUB_DONE_MAGIC = 0x5C7B00D0u;
constexpr uint32_t SCRUB_FLAG_OFF   = 0x3C;
constexpr uint64_t X280_SCRATCH_MEM = 0x0000'2001'0100ULL;   // X280 phys of SCRATCH[0..15]

// Prepare one cluster to run `payload` from X280 physical address `load`:
// writes the payload, sets all four reset vectors, clears suppress-fetch.
inline void stage_payload(Noc& noc, Tile t, uint64_t load, const uint32_t* payload, size_t bytes) {
    for (size_t i = 0; i < bytes / 4; i++) noc.write32(t.x, t.y, load + 4 * i, payload[i]);
    for (int core = 0; core < 4; core++) {
        noc.write32(t.x, t.y, L2CPU_REG_BASE + l2reg::RESET_VECTOR_CORE0 + core * 8,     (uint32_t)load);
        noc.write32(t.x, t.y, L2CPU_REG_BASE + l2reg::RESET_VECTOR_CORE0 + core * 8 + 4, (uint32_t)(load >> 32));
    }
    uint32_t mgmt = noc.read32(t.x, t.y, L2CPU_REG_BASE + l2reg::X280_MANAGEMENT);
    if (mgmt >> 16) noc.write32(t.x, t.y, L2CPU_REG_BASE + l2reg::X280_MANAGEMENT, mgmt & 0xFFFF);
}

// Release all four clusters from reset (one-shot per ASIC reset). Returns the
// L2CPU_RESET value seen beforehand (0x0f expected on a fresh chip).
inline uint32_t release_all_clusters(Noc& noc) {
    uint32_t before = noc.read32(pll::ARC.x, pll::ARC.y, ARC_L2CPU_RESET);
    noc.write32(pll::ARC.x, pll::ARC.y, ARC_L2CPU_RESET, before | 0xF0u);
    return before;
}

// Scrub the LIM of every L2CPU tile by booting the clusters with
// SCRUB_PAYLOAD placed in each tile's scratch registers. Steps the PLL down
// for the release and back up to `run_pll` afterwards. Returns a bitmask of
// tiles that reported done.
inline unsigned scrub_l3_lim(Noc& noc, const pll::Config& run_pll = pll::MHZ_800, double timeout_s = 5.0, bool verbose = true) {
    set_l2cpu_pll(noc, pll::MHZ_200);
    for (int i = 0; i < 4; i++)
        stage_payload(noc, L2CPU_TILE[i], X280_SCRATCH_MEM, SCRUB_PAYLOAD, sizeof(SCRUB_PAYLOAD));
    uint32_t before = release_all_clusters(noc);
    if (verbose) printf("  L2CPU_RESET was 0x%02x; clusters released, waiting for scrub...\n", before);
    set_l2cpu_pll(noc, run_pll);
    unsigned done = 0;
    auto t0 = std::chrono::steady_clock::now();
    while (done != 0xF && std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < timeout_s) {
        for (int i = 0; i < 4; i++)
            if (!(done & (1u << i)) && noc.read32(L2CPU_TILE[i].x, L2CPU_TILE[i].y, X280_SCRATCH_MEM + SCRUB_FLAG_OFF) == SCRUB_DONE_MAGIC)
                done |= 1u << i;
    }
    if (verbose)
        for (int i = 0; i < 4; i++)
            printf("  L2CPU %d: %s\n", i, (done >> i) & 1 ? "LIM scrubbed" : "NO RESPONSE from scrub payload");
    return done;
}

// Does this tile's local GDDR answer? (DRAM tiles can be harvested; then the
// L2CPU wired to that DRAM has no usable local memory.) Writes and reads back
// two words through the uncached alias.
inline bool gddr_present(Noc& noc, Tile t, uint64_t probe_offset = 0x1000'0000ULL) {
    uint64_t a = X280_GDDR_UNCACHED + probe_offset;
    noc.write64(t.x, t.y, a, 0x1234'5678'9ABC'DEF0ULL);
    noc.write64(t.x, t.y, a + 8, 0x0FED'CBA9'8765'4321ULL);
    return noc.read64(t.x, t.y, a) == 0x1234'5678'9ABC'DEF0ULL && noc.read64(t.x, t.y, a + 8) == 0x0FED'CBA9'8765'4321ULL;
}

} // namespace l2dma
