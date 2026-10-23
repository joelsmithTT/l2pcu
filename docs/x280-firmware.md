# X280-driven DMA: `x280/`

`x280_dma_host` boots one L2CPU with `dma_fw.bin`, a 2.2 KiB bare-metal
firmware. The host then sends DMA commands through a mailbox, and the X280
programs the tile's DMA engine itself. The host checks every destination
through a separate path.

## Test cases

| Case | Source | Destination | Mechanism |
|---|---|---|---|
| 1 | LIM, local GDDR | LIM, local GDDR | X280 physical addresses on DMA master 1; crossbar on master 0 |
| 2 | local GDDR | Tensix (1,2) L1, and back | 2 MiB X280 NOC window + outbound DMA TLB |
| 3 | local GDDR, Tensix L1 | DRAM tile (17,12) over the NOC, and back | 2 MiB NOC window |
| 4 | pinned host buffer | GDDR, Tensix L1, host | 128 GiB NOC window at the PCIe tile (19,24), address = IOVA |

## Boot

1. The host steps the PLL down to 200 MHz for the release.
2. The host writes the image through the L2CPU's NOC passthrough into the
   tile's local GDDR at offset 3.75 GiB, plus 16 MiB per tile because tiles
   2 and 3 share a DRAM. It writes through the *cached* alias
   `0x4000_3000_0000 + offset`, so the harts can fetch from it. The image is
   position independent (`-mcmodel=medany`, no absolute pointers), so one
   image serves all four tiles.
3. The host zeroes the mailbox, which sits 64 KiB after the image. Both
   sides access it through the *uncached* alias `0x3000_0000 + offset`, so
   no cache maintenance is needed anywhere. The 64 KiB limit on image plus
   stack is enforced by `link.ld`, and checked again by the host when it
   loads the file.
4. The host points the four reset vectors at the image, clears
   suppress-fetch, and raises the cluster's bit in the ARC `L2CPU_RESET`
   register.
5. Harts 1 to 3 park in `wfi`. Hart 0 installs a trap handler that records
   `mcause`/`mepc`/`mtval` in the mailbox, writes a magic word, ECC-scrubs
   the whole LIM with 64-bit stores (0.6 ms), and enters the command loop.
6. Once it sees the magic word, the host steps the PLL up to 1750 MHz.

Nothing in the boot needs the LIM to be usable beforehand.

## Mailbox protocol

All words are 64-bit; the layout is in `dma_fw.c` and mirrored in
`x280_dma_host.cpp`. The host fills `args[16]`, then writes `cmd`. The
firmware executes the command, fills `status`, `cycles` (mcycle) and
`res[16]`, increments `seq`, and clears `cmd` last.

Commands: `NOP`, `DMA`, `RD64`, `WR64`, `DUMP` (DMAC registers), `PARK`.

A `DMA` command names a source and a destination, each as (kind, x, y,
address, bytes):

| kind | Meaning |
|---|---|
| 0 | X280 physical address, through master 1 and an inbound DMA TLB |
| 1 | NOC location, through a small X280 NOC window, master 0 and an outbound DMA TLB |
| 2 | the same through a large window |
| 3 | raw crossbar address on master 0 |

Transfers larger than one block are chained, using a linked list the
firmware builds in the top 64 KiB of the LIM. A DMA that does not finish
within about 2 s times out rather than hanging (see
[hardware.md](hardware.md#a-hung-noc-read-can-wedge-the-chip)).

## Host buffer

On an IOMMU-enabled system, `PIN_PAGES` with no flags returns an IOVA, and
that IOVA is directly a valid NOC address at the PCIe tile. A 128 GiB
window pointed at `(19,24)` with `local_offset = IOVA >> 37` reaches the
buffer. The host passes the IOVA to the firmware in the mailbox arguments
like any other address. The `NOC_DMA` flag is needed only when the NOC
address has to be known in advance, without a mailbox to pass it through.

## Development loop

```
$EDITOR x280/fw/dma_fw.c
make x280
tt-smi -r
x280/x280_dma_host 0 --keep                           # boot, run the cases, leave the firmware running
x280/x280_dma_host 0 attach dump                      # DMAC registers
x280/x280_dma_host 0 attach rd64 <addr>
x280/x280_dma_host 0 attach wr64 <addr> <value>
x280/x280_dma_host 0 attach dma <sk> <sx> <sy> <sa> <dk> <dx> <dy> <da> <bytes>
x280/x280_dma_host 0 attach hostprobe <x> <y> [kind]   # can the X280 DMA 64 B into a pinned host buffer via (x,y)?
x280/x280_dma_host 0 attach park
```

`--fw <path>` boots a different image without rebuilding the host program.
`attach` does not load firmware; it talks to whatever is already running.
Booting is one-shot (a Blackhole erratum), so the program refuses to boot a
cluster that has already been released.
