# The L2CPU DMA engine

These notes were measured on silicon. Where they disagree with the RTL
headers or the FW team's DV tests, trust these notes.

## The engine

Each L2CPU tile contains a Synopsys **DW_ahb_dmac**:

| Property | Value |
|---|---|
| Channels | 2 |
| Datapath | 256-bit (transfer width 1..32 bytes) |
| Block size | up to 4095 words, so 128 KiB - 32 at 32-byte width |
| Multi-block | linked-list descriptors (LLIs), fetched with 32-bit reads |
| Registers | 64-bit wide, 4 KiB block at X280 physical `0x2008_0000` |
| Handshake | none wired up (lines tied off): memory-to-memory only |
| Interrupts | X280 PLIC global IRQ 4 (ch 0) and 5 (ch 1). The host polls instead |

## Address map

The engine's two AHB masters issue 32-bit addresses. Tables of "DMA TLB"
registers in the L2CPU register block (`0xFFFF_F7FE_FFF1_0000` on the NOC)
widen them:

| Master | Selector bits | TLB registers | Lands in | Granule |
|---|---|---|---|---|
| 0, `EXTERN` | `addr[31:28]` | 16 x `OUTBOUND_DMA_TLB` (20-bit) at `+0x200` | the tile's DDR/NOC crossbar | 256 MiB |
| 1, `L2` | `addr[31:30]` | 4 x `INBOUND_DMA_TLB` (17-bit) at `+0x240` | the X280 front port (coherent) | 1 GiB |

Master 0 sees the crossbar, which is the X280 system-port space minus
`0x3000_0000`:

```
0x0000_0000_0000   local GDDR, 4 GiB
0x0004_0000_0000   small X280 NOC windows, 224 x 2 MiB
0x0804_0000_0000   large X280 NOC windows, 32 x 128 GiB
```

Master 1 sees the whole 47-bit X280 physical map. That includes the L3 LIM
at `0x0800_0000`, the zero device at `0x0A00_0000`, GDDR (uncached at
`0x3000_0000`, cached at `0x4000_3000_0000`) and the NOC windows.

Both tables reset to identity. With no programming, master 0 covers the low
4 GiB of GDDR and master 1 the low 4 GiB of X280 physical space.

### Three different TLBs

Three unrelated address-translation layers sit between the host and a DMA
target. The library names each one separately:

1. **PCIe to NOC windows**, owned by `tt-kmd` (`l2dma::TlbWindow`). The
   host reaches every tile through these, including the DMAC registers.
2. **L2CPU DMA address-expansion TLBs** (`Dmac::set_outbound_tlb`,
   `Dmac::set_inbound_tlb`). These turn the DMAC's 32-bit addresses into
   crossbar or X280 addresses.
3. **X280 NOC TLB windows** (`Dmac::set_noc_small_window`,
   `Dmac::set_noc_large_window`). These are the tile's own 256 windows onto
   the NOC, configured at `0xFFFF_F7FE_FFF0_0000` (16 bytes per small
   window, 12 per large). Master 0 reaches other tiles through them: point a
   window at (x, y, address), then point an outbound DMA TLB at the window's
   crossbar address. `Dmac::extern_noc_small` does both steps.

## Undocumented behaviour

### Register address
The RTL register headers place the DMAC at the NOC "bypass" address
`0xFFFF_F7FE_FFF8_0000`. On silicon that address does not decode: reads
return all ones. The engine does answer through the X280 passthrough at its
physical address `0x2008_0000`, even while the harts are held in reset.

### L3 LIM ECC
The LIM is ECC SRAM, and nothing initialises its ECC bits at power-up. A
DMA read of a line that has never been written returns an AXI error and
increments the L3 `DatECCFail` counter. Writes that arrive through the X280
front port (from the host over the NOC, or from the DMA engine itself) do
not initialise the ECC. Section 18.1.1 of the SiFive manual says ECC SRAM
must be initialised with 64-bit stores from the processor, not from an
external bus.

Until the LIM has been scrubbed, DMA can write it but cannot read it.
Linked-list descriptors live at the top of the LIM by default, so any
multi-block transfer hangs the channel.

`l2cpu_dma_test scrub` boots hart 0 of each tile with a 60-byte payload. The
payload sits in the tile's general-purpose scratch registers, zeroes the
1920 KiB LIM and parks. Harts can be released from reset only once per ASIC
reset, so this is one-shot: afterwards the harts stay parked in `wfi`. The
ECC state survives ASIC resets, so one scrub per power cycle is enough. The
X280 firmware in `x280/` does the same scrub itself on boot.

The payload source:

```asm
    csrr  t0, mhartid
    bnez  t0, park
    auipc t1, 0              # t1 = payload + 8
    lui   a0, 0x08000        # LIM start
    lui   a1, 0x081E0        # LIM end
loop:
    sd    zero, 0(a0)
    addi  a0, a0, 8
    bltu  a0, a1, loop
    fence
    lui   t2, 0x5C7B0
    ori   t2, t2, 0x0D0      # 0x5C7B00D0 = done
    sw    t2, 0x34(t1)       # last scratch word
    fence
park:
    wfi
    j     park
```

### Usable LIM
The usable LIM is 1920 KiB, from `0x0800_0000` to `0x081D_FFFF`: 15 of the
16 L3 ways. Way 0 stays a cache way. A DMA write past the end of the LIM
hangs the channel, so destinations, and the linked-list area (by default
the last 64 KiB), must stay inside it. A read past the 1.5 MiB of a Tensix
L1 hangs the channel in the same way.

### X280 NOC windows
* **`local_offset` is in units of the window size**: `address >> 21` for a
  small window, `address >> 37` for a large one (ISA doc TLBWindows.md, and
  the FW team's `l2_configure_group0_noc_tlb`). If you write the raw address
  instead, only targets below 2 MiB work; anything higher silently lands at
  the wrong address.
* **Reads from DRAM tiles need strict AXI ordering** (window
  `ordering = 1`). DRAM controllers return responses out of order. With the
  default ordering, the AXI bridge stalls after a few hundred KiB. Tensix L1
  answers in order, so the setting makes no difference there.
* **Coordinates are the translated ones**, the same as the host's PCIe
  TLBs use. The L2CPU NIUs have translation enabled, with the same tables as
  the PCIe and DRAM NIUs. For reference: translated (17,12) is raw (9,0),
  (19,24) is raw (2,0), and each L2CPU's translated coordinates equal its
  raw ones.

### A hung NOC read can wedge the chip
If a DMA read addressed to a target that does not respond stays
outstanding, the host goes idle and `tt-kmd`'s idle power management kicks
in. After that, every NOC access returns all-ones and ARC stops answering.
`tt-smi -r` recovers the chip.

### Other gotchas
* **Cached GDDR alias.** A DMA write through `0x4000_3000_0000` lands in L3
  and is not visible through the uncached alias until it is flushed. Use the
  uncached alias unless you want that.
* **Harvested DRAM.** A tile whose DRAM tile was harvested has no working
  local GDDR. `gddr_present()` probes for local GDDR, and the tester skips
  those cases on such tiles.
* **Clock.** The L2CPU PLL comes up at 800 MHz. Stepping it to 1750 MHz
  (`set_l2cpu_pll`, the same sequence as tt-bh-linux's `clock.py`) lasts only
  while some process keeps `/dev/tenstorrent/N` open. When the last handle
  closes, `tt-kmd` tells the ARC firmware the device is idle, and ARC puts
  PLL4 back to 800 MHz. That is why `perf <l2cpu> 1750` sets the clock
  inside its own open.
* **The FW team's `l2cpu/fw/dma_*` tests** are DV-era code. They truncate
  64-bit addresses to 32 bits, and their outbound TLB constants do not match
  the silicon address map. Use them as a register recipe only.

## Performance

Host-driven, L2CPU 0, 32-byte transfer width:

| Transfer | 800 MHz, MiB/s | 1750 MHz, MiB/s |
|---|---|---|
| LIM to LIM (L2/L2) | 620 | |
| LIM to GDDR (L2 to L2 or EXTERN) | 620-650 | |
| GDDR to LIM (L2/L2) | 200 | |
| GDDR to GDDR, EXTERN/EXTERN | 216 | 245 |
| GDDR to GDDR, L2/L2 | 187 | 231 |
| GDDR to GDDR, EXTERN read, L2 write | 247 | 281 |
| GDDR to GDDR, L2 read, EXTERN write | 202 | 258 |
| GDDR to GDDR, both channels, aggregate | 235 | 265 |
| GDDR to Tensix L1 via NOC window | 245 | |
| Tensix L1 to GDDR via NOC window | 77 | |

X280-driven at 1750 MHz: LIM to LIM 1.4 GiB/s; anything that reads GDDR
about 250 MiB/s; reads from Tensix L1 or a DRAM tile over the NOC 65 to
80 MiB/s; reads from host memory 27 MiB/s; writes to host memory 255 MiB/s.

The engine is latency-bound. Burst size makes no difference. Raising the
L2CPU clock from 800 to 1750 MHz gains about 13 percent, and the two masters
differ only in the latency of their paths. Reads are the expensive side: the
engine issues one read burst, waits for all of it to return, then writes.
Running both channels at once helps only a little, so the serialisation is
inside the engine, not in the fabric. This is an engine for bring-up and
housekeeping, not for bandwidth.
