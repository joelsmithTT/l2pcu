# l2cpu — Blackhole L2CPU tools

Each Blackhole L2CPU tile has four SiFive X280 harts and a Synopsys
DW_ahb_dmac. This repo has two ways to drive the DMA engine, plus a probe
that reports which ISA extensions the X280 implements:

* **`host/`**: the host programs the engine directly through `tt-kmd`, with
  the harts held in reset. `l2cpu_dma.hpp` is a single-header C++17 library;
  `l2cpu_dma_test` is its tester and example.
* **`x280/`**: the host boots a small bare-metal firmware on hart 0, and
  the X280 programs the engine itself. It copies between LIM, local GDDR, a
  Tensix L1, a DRAM tile over the NOC and a pinned host buffer over PCIe, and
  the host checks every destination through a separate path.
* **`isa/`**: the host boots a probe firmware that tries each candidate
  instruction and CSR on hart 0 and records which ones trap. It prints the
  resulting ISA string, including the SiFive vendor (Intelligence)
  extensions. Results and method: [docs/x280-isa.md](docs/x280-isa.md).

None of them needs anything beyond `tt-kmd`: the few ioctls they use are
inlined.

## Requirements

* A Blackhole card with `tt-kmd` loaded (`/dev/tenstorrent/N`).
* `tt-smi`, to reset the ASIC between runs that boot harts.
* `g++` with C++17.
* For `x280/` and `isa/`, a RISC-V cross compiler that can target
  `rv64imac`/`lp64`. On Ubuntu: `apt install gcc-riscv64-unknown-elf`.
  `riscv64-linux-gnu-gcc` also works (`make CROSS=riscv64-linux-gnu-`), but
  it produces a different binary from the one these notes were measured with.
* For the host-buffer cases in `x280/`, an IOMMU-enabled system, so that
  `PIN_PAGES` returns an IOVA.

## Build

```
make            # everything
make host       # host/l2cpu_dma_test only, no cross compiler needed
make x280       # x280/x280_dma_host and x280/fw/dma_fw.bin
make isa        # isa/isa_probe and isa/fw/probe_fw.bin
```

Firmware is built from source and loaded at run time: it is not embedded in
the host programs. `x280_dma_host` and `isa_probe` read `fw/dma_fw.bin` and
`fw/probe_fw.bin` from beside their own executables, or the file given with
`--fw <path>`. The build prints the
image size and a hash prefix. The reference build, made with
`riscv64-unknown-elf-gcc` 10.2.0, is 2240 bytes with sha256 starting
`eac9dbc66d8abda5`.

## Run

### Host-driven (`host/`)

```
host/l2cpu_dma_test probe     # read-only register dump of the four engines
tt-smi -r
host/l2cpu_dma_test scrub     # ECC-scrub the L3 LIM (boots a hart per tile, see below)
host/l2cpu_dma_test test      # functional tests on all four tiles
host/l2cpu_dma_test perf 0 1750   # throughput sweep on L2CPU 0 at 1750 MHz
```

`scrub` is needed once per power cycle, not per reset: the LIM's ECC bits are
uninitialised at power-up, and until they are written the LIM cannot be read
by DMA (see [docs/hardware.md](docs/hardware.md#l3-lim-ecc)). Harts can be
released from reset only once per ASIC reset, so `scrub` must come right
after a `tt-smi -r`. The ECC state survives later resets.

With no arguments, `l2cpu_dma_test` runs `probe`. An unrecognised command,
such as `--help`, lists the others: raw NOC peek and poke, PLL control, and
LIM readability checks.

### X280-driven (`x280/`)

```
tt-smi -r                       # harts can be released only once per ASIC reset
x280/x280_dma_host 0            # L2CPU 0..3
```

The firmware scrubs the LIM itself on boot, so no `scrub` step is needed
first. When the run finishes, hart 0 parks in `wfi`. Pass `--keep` to leave
it serving mailbox commands; then
`x280/x280_dma_host 0 attach dma|rd64|wr64|hostprobe|dump|park ...` sends it more
commands without rebooting.

Both programs exit non-zero if any case fails.

### ISA probe (`isa/`)

```
tt-smi -r
isa/isa_probe 0                 # L2CPU 0..3; add --csrs to list every CSR, --sweep to scan custom opcodes
```

It takes a few milliseconds and leaves hart 0 parked. Like the other
programs that boot harts, it needs a fresh `tt-smi -r` each time.

## Layout

```
Makefile
host/
  l2cpu_dma.hpp          library: tt-kmd ABI, NOC access, DMAC registers, the three TLB layers,
                         transfers, LIM scrub, PLL stepping
  l2cpu_dma_test.cpp     tester and example
x280/
  x280_dma_host.cpp      boots dma_fw.bin, runs the test cases, attach mode
  fw/
    dma_fw.c start.S     firmware: trap handler, LIM scrub, mailbox command loop
    link.ld Makefile     -> dma_fw.bin (+ .elf, .lst, .map)
isa/
  isa_probe.cpp          boots probe_fw.bin, prints the report (built on host/l2cpu_dma.hpp)
  probe_abi.h            results layout shared with the firmware
  gen_probes.py          the probe list; regenerates probes.h with llvm-mc (only needed to edit it)
  probes.h               generated instruction table, checked in
  fw/
    probe_fw.c start.S   firmware: trap-and-skip handler, CSR sweep, WARL probes, instruction probes
    link.ld Makefile     -> probe_fw.bin
docs/
  hardware.md            the engine, its address map, undocumented behaviour, performance
  x280-firmware.md       how the X280 side boots and talks to the host, and what went wrong on the way
  x280-isa.md            what the X280 implements, measured, and how the probe works
```

## Using the library

```cpp
#include "l2cpu_dma.hpp"
using namespace l2dma;

Device dev("/dev/tenstorrent/0");
Noc    noc(dev);
Dmac   dma(noc, 0);                  // L2CPU 0

// GDDR offset 1 GiB .. 1 GiB + 256 MiB through outbound slot 1
dma.set_outbound_tlb(1, XBAR_GDDR_BASE + (1ULL << 30));
DmaAddr src = dma.extern_addr(1, XBAR_GDDR_BASE + (1ULL << 30));
DmaAddr dst = { Master::L2, 0x0810'0000 };   // LIM + 1 MiB, identity inbound TLB 0

XferResult r = dma.copy(/*channel*/ 0, src, dst, 64 * 1024);
printf("%s %.1f MiB/s\n", r.ok ? "ok" : "failed", r.mib_per_s());
```

`copy()` is `start()` followed by `wait()`. Call those two directly to keep
both channels busy. Transfers larger than one block are chained
automatically, using a linked list built in the LIM (`XferOptions::lli_*`
moves it). When something goes wrong, `Dmac::dump()` prints the registers.

## Troubleshooting

* **"already released from reset"**: run `tt-smi -r`. Every boot of a hart
  (`scrub`, `x280_dma_host`, `isa_probe`) uses up the one release an ASIC
  reset allows.
* **A DMA channel never completes**: likely causes are a destination past the
  end of the LIM, a DMA read of LIM that has not been scrubbed, or a read
  from a DRAM tile without strict ordering. All are covered in
  [docs/hardware.md](docs/hardware.md).
* **Every NOC read returns all-ones and ARC stops answering**: a DMA read
  hung on the NOC and the device went idle. `tt-smi -r` recovers it.
