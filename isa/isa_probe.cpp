// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
// SPDX-License-Identifier: GPL-2.0-only
//
// isa_probe - report which RISC-V extensions a Blackhole X280 implements, by
// booting probe_fw on hart 0 of one L2CPU and having it try them.
//
//   isa_probe [l2cpu 0..3] [--fw <probe_fw.bin>] [--csrs] [--sweep]
//
//   --fw     firmware image, default fw/probe_fw.bin next to this executable
//   --csrs   list every implemented CSR with its value
//   --sweep  also try every R-type encoding in the custom-0..3 opcode spaces
//            and list the ones that do not trap (finds vendor instructions
//            that are not in the probe table)
//
// Harts can be released only once per ASIC reset: run `tt-smi -r` first.

#include "../host/l2cpu_dma.hpp"

#include <cinttypes>
#include <map>
#include <strings.h>
#include <thread>

extern "C" {
#include "probe_abi.h"
#include "probes.h"
}

using namespace l2dma;

namespace {

struct Probe { const char* ext; const char* text; uint32_t insn; probe_kind kind; unsigned sew; };
#define PROBE_ENTRY(ext, text, insn, kind, sew) { ext, text, insn, kind, sew },
const Probe PROBES[] = { PROBE_LIST(PROBE_ENTRY) };
constexpr size_t NPROBES = sizeof(PROBES) / sizeof(PROBES[0]);

// Same placement as x280_dma_host: GDDR offset 3.75 GiB + 16 MiB per tile,
// code through the cached alias, results through the uncached alias.
uint64_t fw_gddr_offset(int i) { return 0xF000'0000ULL + (uint64_t)i * 0x100'0000; }

std::string default_fw_path() {
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return "fw/probe_fw.bin";
    std::string exe(buf, n);
    return exe.substr(0, exe.rfind('/') + 1) + "fw/probe_fw.bin";
}

std::vector<uint8_t> load_firmware(const std::string& path) {
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) throw std::system_error(errno, std::generic_category(), "open firmware " + path + " (build it with `make isa` at the top of the repo)");
    std::vector<uint8_t> img;
    uint8_t buf[4096];
    for (size_t n; (n = fread(buf, 1, sizeof(buf), f)) > 0;) img.insert(img.end(), buf, buf + n);
    fclose(f);
    if (img.empty() || img.size() > RESULTS_OFFSET)
        throw std::runtime_error("firmware " + path + " is " + std::to_string(img.size()) + " bytes; expected 1.." + std::to_string(RESULTS_OFFSET));
    img.resize((img.size() + 7) & ~size_t(7), 0);
    return img;
}

std::unique_ptr<Device> open_blackhole() {
    for (auto& p : Device::enumerate()) {
        auto d = std::make_unique<Device>(p);
        if (d->is_blackhole()) return d;
    }
    throw std::runtime_error("no Blackhole device found under /dev/tenstorrent");
}

const char* state_name(uint64_t s) {
    switch (s) {
    case ST_BOOT: return "boot"; case ST_CSRS: return "CSR sweep"; case ST_WARL: return "WARL probes";
    case ST_PROBES: return "instruction probes"; case ST_SWEEP: return "opcode sweep"; case ST_DONE: return "done";
    case ST_TRAPPED: return "trapped"; default: return "?";
    }
}

std::string cause_name(uint32_t c) {
    switch (c) {
    case NO_TRAP: return "-";
    case 0: return "insn misaligned"; case 1: return "insn access fault"; case 2: return "illegal instruction";
    case 3: return "breakpoint"; case 4: return "load misaligned"; case 5: return "load access fault";
    case 6: return "store/AMO misaligned"; case 7: return "store/AMO access fault"; case 11: return "ecall";
    default: return "mcause " + std::to_string(c);
    }
}

// Boot probe_fw on cluster `idx` and wait for it to finish. Returns the
// results as far as the firmware got.
probe_results run_firmware(Noc& noc, int idx, const std::vector<uint8_t>& fw, uint64_t flags) {
    const Tile t = L2CPU_TILE[idx];
    uint32_t reset = noc.read32(pll::ARC.x, pll::ARC.y, ARC_L2CPU_RESET);
    if ((reset >> 4) & (1u << idx))
        throw std::runtime_error("L2CPU " + std::to_string(idx) + " was already released from reset; harts can only be "
                                 "booted once per ASIC reset. Run `tt-smi -r` and retry.");
    if (!gddr_present(noc, t))
        throw std::runtime_error("L2CPU " + std::to_string(idx) + " has no working local GDDR (harvested DRAM?); try another L2CPU");

    const uint64_t code = X280_GDDR_CACHED + fw_gddr_offset(idx);
    const uint64_t res = X280_GDDR_UNCACHED + fw_gddr_offset(idx) + RESULTS_OFFSET;
    printf("Booting probe_fw (%zu bytes) on L2CPU %d (%u,%u), hart 0\n", fw.size(), idx, t.x, t.y);
    set_l2cpu_pll(noc, pll::MHZ_200);
    noc.fill64(t.x, t.y, res, 0, sizeof(probe_results));
    noc.write64(t.x, t.y, res + offsetof(probe_results, flags), flags);
    stage_payload(noc, t, code, (const uint32_t*)fw.data(), fw.size());
    noc.write32(pll::ARC.x, pll::ARC.y, ARC_L2CPU_RESET, reset | (0x10u << idx));

    auto rd = [&](size_t off) { return noc.read64(t.x, t.y, res + off); };
    auto t0 = std::chrono::steady_clock::now();
    auto since = [](std::chrono::steady_clock::time_point a) {
        return std::chrono::duration<double>(std::chrono::steady_clock::now() - a).count();
    };
    while (rd(offsetof(probe_results, magic)) != PROBE_MAGIC) {
        if (since(t0) > 5.0) throw std::runtime_error("firmware did not come up (no magic in the results area)");
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    set_l2cpu_pll(noc, pll::MHZ_800);

    // A step that takes more than 5 s without moving on has hung the hart.
    uint64_t last_state = 0, last_progress = ~0ULL;
    auto last_change = std::chrono::steady_clock::now();
    for (;;) {
        uint64_t st = rd(offsetof(probe_results, state)), pr = rd(offsetof(probe_results, progress));
        if (st == ST_DONE || st == ST_TRAPPED) break;
        if (st != last_state || pr != last_progress) { last_state = st; last_progress = pr; last_change = std::chrono::steady_clock::now(); }
        else if (since(last_change) > 5.0) {
            std::string what = state_name(st) + std::string(" step ") + std::to_string(pr);
            if (st == ST_PROBES && pr < NPROBES) what += std::string(" (") + PROBES[pr].text + ")";
            if (st == ST_SWEEP) {
                char w[16]; snprintf(w, sizeof(w), "0x%08x", (unsigned)SWEEP_WORD((unsigned)(pr % SWEEP_COUNT)));
                what += std::string(" (encoding ") + w + ")";
            }
            fprintf(stderr, "warning: the hart stopped making progress in %s; reporting what it finished\n", what.c_str());
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    printf("  finished in %.2f s\n\n", since(t0));

    probe_results r;
    noc.read(t.x, t.y, res, &r, sizeof(r));
    return r;
}

// ---------------------------------------------------------------------------
// Report
// ---------------------------------------------------------------------------

const std::map<unsigned, std::string>& csr_names() {
    static const std::map<unsigned, std::string> m = [] {
        std::map<unsigned, std::string> n = {
            {0x001, "fflags"}, {0x002, "frm"}, {0x003, "fcsr"},
            {0x008, "vstart"}, {0x009, "vxsat"}, {0x00A, "vxrm"}, {0x00F, "vcsr"}, {0x015, "seed"}, {0x017, "jvt"},
            {0xC00, "cycle"}, {0xC01, "time"}, {0xC02, "instret"}, {0xC20, "vl"}, {0xC21, "vtype"}, {0xC22, "vlenb"},
            {0x100, "sstatus"}, {0x104, "sie"}, {0x105, "stvec"}, {0x106, "scounteren"}, {0x10A, "senvcfg"},
            {0x10C, "sstateen0"}, {0x140, "sscratch"}, {0x141, "sepc"}, {0x142, "scause"}, {0x143, "stval"},
            {0x144, "sip"}, {0x14D, "stimecmp"}, {0x150, "siselect"}, {0x151, "sireg"}, {0x180, "satp"},
            {0x15C, "stopei"}, {0xDB0, "stopi"}, {0x5A8, "scontext"}, {0xDA0, "scountovf"},
            {0x308, "mvien"}, {0x309, "mvip"}, {0x35C, "mtopei"}, {0xFB0, "mtopi"},
            {0x307, "mtvt"}, {0x345, "mnxti"}, {0x347, "mintthresh"}, {0x348, "mscratchcsw"}, {0xFB1, "mintstatus"},
            // SiFive, as named in the U7/X2-series core manuals
            {0x7C0, "sifive bpm"}, {0x7C1, "sifive featdis"},
            {0x600, "hstatus"}, {0x680, "hgatp"}, {0x200, "vsstatus"},
            {0xF11, "mvendorid"}, {0xF12, "marchid"}, {0xF13, "mimpid"}, {0xF14, "mhartid"}, {0xF15, "mconfigptr"},
            {0x300, "mstatus"}, {0x301, "misa"}, {0x302, "medeleg"}, {0x303, "mideleg"}, {0x304, "mie"},
            {0x305, "mtvec"}, {0x306, "mcounteren"}, {0x30A, "menvcfg"}, {0x30C, "mstateen0"}, {0x310, "mstatush"},
            {0x320, "mcountinhibit"}, {0x340, "mscratch"}, {0x341, "mepc"}, {0x342, "mcause"}, {0x343, "mtval"},
            {0x344, "mip"}, {0x34A, "mtinst"}, {0x34B, "mtval2"}, {0x350, "miselect?"}, {0x351, "mireg?"},
            {0x352, "mireg2?"}, {0x353, "mireg3?"},
            {0x747, "mseccfg"}, {0x7A0, "tselect"}, {0x7A1, "tdata1"}, {0x7A2, "tdata2"}, {0x7A3, "tdata3"},
            {0x7A4, "tinfo"}, {0x7A5, "tcontrol"}, {0x7A8, "mcontext"},
            {0x7B0, "dcsr"}, {0x7B1, "dpc"}, {0x7B2, "dscratch0"}, {0x7B3, "dscratch1"},
            {0xB00, "mcycle"}, {0xB02, "minstret"},
        };
        auto add = [&](unsigned csr, std::string s) { n[csr] = s; };
        for (int i = 3; i < 32; i++) {
            add(0xC00 + i, "hpmcounter" + std::to_string(i));
            add(0xB00 + i, "mhpmcounter" + std::to_string(i));
            add(0x320 + i, "mhpmevent" + std::to_string(i));
        }
        for (int i = 0; i < 16; i++) add(0x3A0 + i, "pmpcfg" + std::to_string(i));
        for (int i = 0; i < 64; i++) add(0x3B0 + i, "pmpaddr" + std::to_string(i));
        return n;
    }();
    return m;
}

// CSR numbers the privileged spec reserves for custom use.
bool custom_csr(unsigned c) {
    static const unsigned ranges[][2] = {
        {0x800, 0x8FF}, {0xCC0, 0xCFF},                                   // user
        {0x5C0, 0x5FF}, {0x9C0, 0x9FF}, {0xDC0, 0xDFF},                   // supervisor
        {0x6C0, 0x6FF}, {0xAC0, 0xAFF}, {0xEC0, 0xEFF},                   // hypervisor
        {0x7C0, 0x7FF}, {0xBC0, 0xBFF}, {0xFC0, 0xFFF},                   // machine
    };
    for (auto& r : ranges) if (c >= r[0] && c <= r[1]) return true;
    return false;
}

bool csr_present(const probe_results& r, unsigned c) { return (r.csr_present[c / 64] >> (c % 64)) & 1; }

std::string vtype_list(uint32_t mask) {
    std::string s;
    for (unsigned i = 0; i < NUM_VTYPES; i++)
        if ((mask >> i) & 1) s += (s.empty() ? "" : " ") + std::string("e") + std::to_string(VTYPE_SEW(i)) + "m" + std::to_string(VTYPE_LMUL(i));
    return s;
}

// How many probes ran to completion (all of them unless the hart hung or
// trapped during that phase).
size_t probes_completed(const probe_results& r) {
    uint64_t phase = r.state == ST_TRAPPED ? r.trap_state : r.state;
    if (phase < ST_PROBES) return 0;
    if (phase == ST_PROBES) return std::min<size_t>(r.progress, NPROBES);
    return std::min<size_t>(r.nprobes, NPROBES);
}

// Did probe p succeed in the way that implies its extension?
bool probe_implies(const probe_results& r, size_t p) {
    if (PROBES[p].kind != P_VECTOR) return r.probe_ok[p] & 1;
    for (unsigned i = 0; i < NUM_VTYPES; i++)
        if (((r.probe_ok[p] >> i) & 1) && (PROBES[p].sew == 0 || VTYPE_SEW(i) == PROBES[p].sew)) return true;
    return false;
}

// Canonical-ish ISA string order: single letters, then Z by category letter,
// then S, then X.
int ext_rank(const std::string& e) {
    static const std::string order = "imafdqlcbkjtpvh";
    if (e.size() == 1) return (int)order.find((char)tolower(e[0]));
    char c0 = (char)tolower(e[0]);
    if (c0 == 'z') return 100 + (int)order.find((char)tolower(e[1]));
    if (c0 == 's') return 200;
    return 300;
}

void report(const probe_results& r, bool list_csrs) {
    printf("Identification\n");
    printf("  mvendorid   0x%" PRIx64 "%s\n", r.mvendorid, r.mvendorid == 0x489 ? "  (SiFive)" : "");
    printf("  marchid     0x%" PRIx64 "\n", r.marchid);
    printf("  mimpid      0x%" PRIx64 "\n", r.mimpid);
    printf("  mhartid     %" PRIu64 "\n", r.mhartid);
    std::string letters;
    for (int b = 0; b < 26; b++) if ((r.misa >> b) & 1) letters += (char)('A' + b);
    printf("  misa        0x%016" PRIx64 "  RV%d %s\n", r.misa, r.misa >> 62 == 2 ? 64 : r.misa >> 62 == 1 ? 32 : 128, letters.c_str());
    printf("  mconfigptr  0x%" PRIx64 "%s\n", r.mconfigptr, csr_present(r, 0xF15) ? "" : "  (CSR not implemented)");
    if (r.vlenb) {
        printf("  VLEN        %" PRIu64 " bits (vlenb = %" PRIu64 ")\n", r.vlenb * 8, r.vlenb);
        printf("  vsetvl      accepts %s\n", vtype_list((uint32_t)r.vtype_ok).c_str());
    }
    printf("\n");

    // Per-probe detail
    printf("Instruction probes (yes = executed without trapping)\n");
    printf("  %-20s %-38s %s\n", "extension", "instruction", "result");
    const size_t done = probes_completed(r);
    for (size_t p = 0; p < done; p++) {
        std::string res;
        if (PROBES[p].kind == P_VECTOR) {
            res = r.probe_ok[p] ? "yes at " + vtype_list(r.probe_ok[p]) : "no";
            if (PROBES[p].sew) res += "  (implies " + std::string(PROBES[p].ext) + " at e" + std::to_string(PROBES[p].sew) + ")";
        } else {
            res = (r.probe_ok[p] & 1) ? "yes" : "no";
        }
        if (r.probe_cause[p] != NO_TRAP && r.probe_cause[p] != 2) res += "  [trapped: " + cause_name(r.probe_cause[p]) + "]";
        printf("  %-20s %-38s %s\n", PROBES[p].ext, PROBES[p].text, res.c_str());
    }
    if (done < NPROBES) printf("  (stopped after %zu of %zu probes)\n", done, NPROBES);
    printf("\n");

    // Per-extension summary: present if every probe for it worked, partial if
    // only some did.
    struct Tally { unsigned ok = 0, total = 0; std::string failed; };
    std::map<std::string, Tally> ext;
    for (size_t p = 0; p < done; p++) {
        Tally& t = ext[PROBES[p].ext];
        t.total++;
        if (probe_implies(r, p)) t.ok++;
        else t.failed += (t.failed.empty() ? "" : ", ") + std::string(PROBES[p].text);
    }
    // Extensions visible only through CSRs, with the evidence.
    std::map<std::string, std::string> from_csrs;
    auto csrs_all = [&](std::initializer_list<unsigned> cs) { for (unsigned c : cs) if (!csr_present(r, c)) return false; return true; };
    from_csrs["Zicsr"] = "CSR instructions work";
    if (csrs_all({0x14D})) from_csrs["Sstc"] = "stimecmp";
    if (csrs_all({0xDA0})) from_csrs["Sscofpmf"] = "scountovf";
    if (csrs_all({0x30C})) from_csrs["Smstateen"] = "mstateen0";
    if (csrs_all({0x350, 0x351, 0x35C, 0xFB0})) from_csrs["Smaia"] = "miselect, mireg, mtopei, mtopi";
    if (csrs_all({0x7A0, 0x7A1})) from_csrs["Sdtrig"] = "tselect, tdata1";
    if (r.satp_modes & (1u << 8))  from_csrs["Sv39"] = "satp MODE 8 accepted";
    if (r.satp_modes & (1u << 9))  from_csrs["Sv48"] = "satp MODE 9 accepted";
    if (r.satp_modes & (1u << 10)) from_csrs["Sv57"] = "satp MODE 10 accepted";
    if (r.menvcfg_ok && (r.menvcfg_rb >> 62) & 1) from_csrs["Svpbmt"] = "menvcfg.PBMTE writable";
    if (r.menvcfg_ok && (r.menvcfg_rb >> 61) & 1) from_csrs["Svadu"] = "menvcfg.ADUE writable";
    if (r.vlenb) from_csrs["Zvl" + std::to_string(r.vlenb * 8) + "b"] = "vlenb = " + std::to_string(r.vlenb);
    for (auto& [e, why] : from_csrs) { (void)why; ext[e].ok = ext[e].total = 1; }

    std::vector<std::string> have, missing, partial;
    for (auto& [e, t] : ext) (t.ok == t.total ? have : t.ok ? partial : missing).push_back(e);
    auto by_rank = [](const std::string& a, const std::string& b) {
        int ra = ext_rank(a), rb = ext_rank(b);
        return ra != rb ? ra < rb : strcasecmp(a.c_str(), b.c_str()) < 0;
    };
    std::sort(have.begin(), have.end(), by_rank);
    std::sort(missing.begin(), missing.end(), by_rank);
    std::sort(partial.begin(), partial.end(), by_rank);

    std::string isa = r.misa >> 62 == 2 ? "rv64i" : "rv32i";
    std::string priv;
    for (auto& e : have) {
        // S/H are privilege modes and S* extensions are not -march material.
        if (e == "S" || e == "H" || toupper(e[0]) == 'S') { priv += (priv.empty() ? "" : " ") + e; continue; }
        isa += e.size() == 1 ? e : "_" + e;
    }
    for (auto& c : isa) c = (char)tolower(c);
    printf("ISA string from probes (accepted as -march by clang 20):\n  %s\n", isa.c_str());
    printf("Privileged extensions:\n  %s\n\n", priv.c_str());
    printf("Present:  ");
    for (auto& e : have) printf("%s ", e.c_str());
    printf("\nPartial:  ");
    for (auto& e : partial) printf("%s (not: %s) ", e.c_str(), ext[e].failed.c_str());
    printf("\nAbsent:   ");
    for (auto& e : missing) printf("%s ", e.c_str());
    printf("\nFrom CSRs rather than instructions:\n");
    for (auto& [e, why] : from_csrs) printf("  %-10s %s\n", e.c_str(), why.c_str());
    printf("\n");

    // Privileged details
    printf("Privileged\n");
    if (r.satp_ok) {
        printf("  satp modes  ");
        static const char* names[16] = { "Bare", "?1", "?2", "?3", "?4", "?5", "?6", "?7", "Sv39", "Sv48", "Sv57", "Sv64" };
        printf("Bare ");
        for (int m = 1; m < 16; m++) if ((r.satp_modes >> m) & 1) printf("%s ", names[m] ? names[m] : "?");
        if (r.satp_modes & (1u << 8)) {
            int bits = 0; for (uint64_t a = r.satp_asid_rb; a & 1; a >>= 1) bits++;
            printf(" ASID bits %d%s", bits, bits ? "" : " (no ASIDs)");
        }
        printf("\n");
    } else {
        printf("  satp        not accessible (no S-mode?)\n");
    }
    if (r.menvcfg_ok) {
        uint64_t m = r.menvcfg_rb;
        printf("  menvcfg     writable bits 0x%016" PRIx64 ":%s%s%s%s%s%s%s\n", m,
               (m & 1) ? " FIOM" : "", ((m >> 4) & 3) ? " CBIE" : "", ((m >> 6) & 1) ? " CBCFE" : "",
               ((m >> 7) & 1) ? " CBZE" : "", ((m >> 61) & 1) ? " ADUE" : "", ((m >> 62) & 1) ? " PBMTE" : "",
               ((m >> 63) & 1) ? " STCE" : "");
    } else {
        printf("  menvcfg     not implemented\n");
    }
    if (r.mcounteren_ok) {
        int hpm = 0; for (int i = 3; i < 32; i++) if ((r.mcounteren_rb >> i) & 1) hpm++;
        printf("  mcounteren  writable bits 0x%08" PRIx64 " (%d hpmcounters)\n", r.mcounteren_rb, hpm);
    }
    printf("  PMP         %" PRIu64 " entries", r.pmp_entries);
    if (r.pmpaddr0_rb) {
        int g = 0; while (!((r.pmpaddr0_rb >> g) & 1)) g++;
        printf(", granularity %llu bytes", 1ULL << (g + 2));
    } else if (r.pmp_entries) {
        printf(" (granularity not measured: pmp0cfg was in use)");
    }
    printf("\n\n");

    // CSRs
    unsigned n = 0;
    std::vector<unsigned> custom;
    for (unsigned c = 0; c < 4096; c++) if (csr_present(r, c)) { n++; if (custom_csr(c)) custom.push_back(c); }
    printf("CSRs: %u implemented, %zu in the custom ranges\n", n, custom.size());
    for (unsigned c : custom) {
        auto it = csr_names().find(c);
        printf("  0x%03x %-14s 0x%016" PRIx64 "\n", c, it != csr_names().end() ? it->second.c_str() : "", r.csr_value[c]);
    }
    if (list_csrs) {
        printf("All implemented CSRs:\n");
        for (unsigned c = 0; c < 4096; c++) {
            if (!csr_present(r, c)) continue;
            auto it = csr_names().find(c);
            printf("  0x%03x %-14s 0x%016" PRIx64 "\n", c, it != csr_names().end() ? it->second.c_str() : "", r.csr_value[c]);
        }
    }
    printf("\n");
}

void report_sweep(const probe_results& r) {
    printf("Custom-opcode sweep: R-type encodings (rd=x24/v24, rs1=rs2=x16/v16) that did not trap at e8, e16 or e32\n");
    static const char* space[4] = { "custom-0", "custom-1", "custom-2", "custom-3" };
    unsigned found = 0;
    for (unsigned sp = 0; sp < SWEEP_OPCODES; sp++) {
        for (unsigned f3 = 0; f3 < 8; f3++) {
            std::string ok, other;
            for (unsigned f7 = 0; f7 < 128; f7++) {
                unsigned i = (sp << 10) | (f3 << 7) | f7;
                char b[8]; snprintf(b, sizeof(b), " %02x", f7);
                if ((r.sweep_ok[i / 64] >> (i % 64)) & 1) { ok += b; found++; }
                else if ((r.sweep_other[i / 64] >> (i % 64)) & 1) { other += b; found++; }
            }
            if (ok.empty() && other.empty()) continue;
            printf("  %s (opcode 0x%02x) funct3=%u\n", space[sp], SWEEP_OPCODE(sp), f3);
            if (!ok.empty()) printf("    executes, funct7:%s\n", ok.c_str());
            if (!other.empty()) printf("    trapped with a non-illegal-instruction cause, funct7:%s\n", other.c_str());
            for (auto& p : PROBES) {
                if ((p.insn & 0x707Fu) != ((f3 << 12) | SWEEP_OPCODE(sp))) continue;
                unsigned i = (sp << 10) | (f3 << 7) | (p.insn >> 25);
                bool hit = (r.sweep_ok[i / 64] >> (i % 64)) & 1;
                printf("    funct7 %02x is %-34s %s\n", p.insn >> 25, p.text, hit ? "(hit)" : "(not hit)");
            }
        }
    }
    if (!found) printf("  none\n");
    printf("  funct7 bit 0 is the vector mask bit (vm) for vector-format instructions.\n"
           "  Disassemble an encoding with e.g. `echo 0x5b 0x24 0x88 0xf7 | llvm-mc --disassemble -triple=riscv64 -mattr=+v,+xsfvqmaccqoq`\n\n");
}

} // namespace

int main(int argc, char** argv) {
    std::string fw_path = default_fw_path();
    bool list_csrs = false, sweep = false;
    int idx = 0;
    for (int i = 1; i < argc; i++) {
        std::string a = argv[i];
        if (a == "--fw" && i + 1 < argc) fw_path = argv[++i];
        else if (a == "--csrs") list_csrs = true;
        else if (a == "--sweep") sweep = true;
        else if (a.size() == 1 && a[0] >= '0' && a[0] <= '3') idx = a[0] - '0';
        else {
            fprintf(stderr, "usage: %s [l2cpu 0..3] [--fw <probe_fw.bin>] [--csrs] [--sweep]\n", argv[0]);
            return 2;
        }
    }
    try {
        std::vector<uint8_t> fw = load_firmware(fw_path);
        auto dev = open_blackhole();
        printf("Using %s (%s)\n", dev->path().c_str(), dev->bdf().c_str());
        Noc noc(*dev);
        probe_results r = run_firmware(noc, idx, fw, sweep ? FLAG_SWEEP : 0);
        if (r.state == ST_TRAPPED)
            printf("Firmware trapped during %s: mcause=0x%" PRIx64 " mepc=0x%" PRIx64 " mtval=0x%" PRIx64 "\n\n",
                   state_name(r.trap_state), r.trap_cause, r.trap_epc, r.trap_tval);
        report(r, list_csrs);
        if (sweep) report_sweep(r);
        return r.state == ST_DONE ? 0 : 1;
    } catch (const std::exception& e) {
        fprintf(stderr, "error: %s\n", e.what());
        return 1;
    }
}
