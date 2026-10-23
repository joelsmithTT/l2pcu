# What the Blackhole X280 implements

These results come from running `isa/isa_probe` on all four L2CPUs of one
Blackhole card (2026-10-23). All four gave identical results.

```
mvendorid  0x489 (SiFive)    marchid 0x8000000000000007    mimpid 0x6220425
misa       0x8000000000b4112d  RV64 ACDFIMSUVX
VLEN       512
```

ISA string, accepted as `-march` by clang 20:

```
rv64imafdcv_zicsr_zifencei_zihpm_zfh_zfhmin_zba_zbb_zvfbfmin_zvfh_zvfhmin_zvl512b_xsfvfnrclipxfqf_xsfvqmaccqoq_xsifivecflushdlone
```

Privileged: S and U modes, no H. Sv39 and Sv48, but no Sv57 and no ASIDs
(satp's ASID field reads back 0). Also Sdtrig (tselect/tdata1, mcontrol6
triggers), Sscofpmf (scountovf), 4 hpmcounters and 8 PMP entries. There is
no `menvcfg`, which fits privileged spec 1.11: no Svpbmt, no Sstc, and no
Zicbo*.

## SiFive Intelligence Extensions

"SiFive Intelligence Extensions" is SiFive's marketing name for the
ML-oriented additions that the X280 carries on top of RVV 1.0. The public
names and encodings come from SiFive's vendor-extension specs, as
implemented in LLVM (all version 1.0 there):

| LLVM name | What it is | Blackhole X280 |
|---|---|---|
| `XSfvqmaccqoq` | int8 matrix multiply-accumulate, 4x8 by 8x4 (`sf.vqmacc[u,us,su].4x8x4`) | **yes**, all four signedness variants, unmasked only, at SEW=8 |
| `XSfvqmaccdod` | int8 matrix multiply-accumulate, 2x8 by 8x2 (`sf.vqmacc*.2x8x2`) | no |
| `XSfvfnrclipxfqf` | FP32 to int8 narrowing clip (`sf.vfnrclip.x[u].f.qf`) | **yes**, masked and unmasked, at SEW=8 |
| `XSfvfwmaccqqq` | BF16 matrix multiply-accumulate, 4x4x4 (`sf.vfwmacc.4x4x4`) | no |
| `XSfvcp` | VCIX, the vector coprocessor interface (`sf.vc.*`) | no (illegal instruction) |
| `Zvfh` / `Zvfhmin` | vector FP16 arithmetic / conversions | **yes** |
| `Zvfbfmin` | vector BF16 conversions | **yes** |
| `Zvfbfwma` | vector BF16 widening multiply-add | no |
| `Zfh` / `Zfhmin` | scalar FP16 | **yes** |
| `Zfbfmin` | scalar BF16 conversion | no |

SiFive cache-control instructions: `sf.cflush.d.l1` works and
`sf.cdiscard.d.l1` does not. Because Zicbom is absent, `sf.cflush.d.l1` is
how to write back the L1 D-cache.

LLVM's `-mcpu=sifive-x280` enables `zfh`, `zvfh`, `zba`, `zbb` and
`zvl512b`, but none of the vendor extensions. To get the vendor intrinsics
(`__riscv_sf_vqmacc_4x8x4`, `__riscv_sf_vfnrclip_x_f_qf`, and so on), add
`_xsfvqmaccqoq_xsfvfnrclipxfqf` to `-march`, or use the string above. GCC 10
does not know any of them.

The device tree doesn't list these extensions, which is why they look
undocumented. `mconfigptr` is not implemented, so there is no configuration
structure to read them from either. Probing is the only way to find out.

## Absent

Q, H, Zicbom, Zicboz, Zicond, Zimop, Zcmop, Zcb, Zabha, Zacas, Zawrs, Zfa,
Zfbfmin, Zbc, Zbs, all of scalar and vector crypto (Zbk*, Zk*, Zvk*, Zvbb,
Zvbc), Zvfbfwma, Svinval, XSfvqmaccdod, XSfvfwmaccqqq, XSfvcp,
XSiFivecdiscarddlone.

`rdtime` traps (illegal instruction). That is normal: `time` is meant to be
emulated by M-mode firmware from the CLINT's `mtime`. `rdcycle`, `rdinstret`
and `hpmcounter3` work.

## CSRs

159 CSR numbers are implemented. Four of them are in custom ranges:

| CSR | Value after reset | Notes |
|---|---|---|
| `0x7C0` | 0 | branch-prediction mode in SiFive U7/X2-series manuals |
| `0x7C1` | `0x3028f` | "feature disable" in those manuals; several bits set at reset |
| `0x7C8` | 0 | unknown |
| `0xFFC` | 0 | unknown, machine read-only |

`0x350` to `0x353` also read without trapping. Those are the Smcsrind/AIA
numbers (`miselect`, `mireg`, `mireg2`, `mireg3`), but the rest of AIA
(`mtopi`, `mtopei`) is absent, so the probe does not claim either
extension. `isa_probe --csrs` lists every implemented CSR with its value.

## How the probe works

`isa_probe` boots `probe_fw.bin` on hart 0 of one L2CPU, in the same way
that `x280_dma_host` boots `dma_fw` (see [x280-firmware.md](x280-firmware.md)).
The firmware then works through the following phases in M-mode:

1. **CSR sweep.** It reads every CSR number from 0 to 0xFFF with `csrrs rd,
   csr, x0`, which has no side effects, and records which ones trap.
2. **WARL probes.** For `satp` (each MODE value, then the ASID width),
   `menvcfg`, `mcounteren` and the PMP address registers, it writes a
   pattern, reads it back and restores the old value. The PMP step skips
   entries whose config byte is non-zero.
3. **Instruction probes.** It runs each instruction listed in
   `isa/gen_probes.py` from a small code buffer: the instruction, then
   `ret`, made visible to instruction fetch with `fence.i`. The trap handler
   in `start.S` counts any trap, records `mcause` and skips the instruction.
   Vector instructions are tried at SEW 8/16/32/64 and LMUL 1/2. Some only
   imply an extension at one SEW (`vfwmacc.vv` needs Zvfh only at e16), and
   the table records which.
4. **Opcode sweep** (`--sweep` only). It tries all 4096 R-type encodings in
   custom-0 to custom-3 at e8, e16 and e32, to catch vendor instructions
   LLVM doesn't know about. On this part it finds exactly the
   `sf.vqmacc*.4x8x4` and `sf.vfnrclip.*` encodings and nothing else.

The host polls a results structure in uncached GDDR. If the firmware stops
making progress for 5 s, the host names the step it was on and reports
whatever finished before it.

### Limits

* "Executes without trapping" shows that an instruction is decoded and
  accepted. It does not check the results. The int8 matrix instructions in
  particular deserve a numerical test before anyone relies on their
  semantics.
* The opcode sweep covers R-type layouts in the four custom opcodes only.
  Vendor instructions hidden in reserved OP-V funct6 slots or other major
  opcodes would not be found.
* Hints such as `pause`, `prefetch.*` and the NTL hints execute as no-ops
  whether or not the extension is implemented, so they can't be detected.
  Zvkt and Svnapot would need more than a trap test.

### Changing the probe list

Edit `PROBES` in `isa/gen_probes.py`, then run `python3 isa/gen_probes.py`.
The script needs llvm-mc from LLVM 18 or later for the SiFive mnemonics; it
was last run with LLVM 20.1.8. It rewrites `isa/probes.h`, which is checked
in, so ordinary builds need only the gcc cross compiler.
