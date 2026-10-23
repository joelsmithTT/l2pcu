#!/usr/bin/env python3
# SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
# SPDX-License-Identifier: GPL-2.0-only
"""Generate probes.h, the instruction table shared by fw/probe_fw.c and
isa_probe.cpp, by assembling each probe with llvm-mc.

The checked-in probes.h is what the build uses, so only someone editing the
list needs LLVM (18 or later, for the SiFive vendor extensions):

    python3 gen_probes.py [--llvm-mc /usr/lib/llvm-20/bin/llvm-mc]

Register conventions (the firmware relies on them): integer results go to t3
and t4 is the second operand, a0 points at a 64-byte-aligned scratch line,
FP uses ft0, vectors use v8 (dest), v16 and v24 (sources), so the groups stay
aligned up to LMUL=8.
"""
import argparse, pathlib, re, shutil, subprocess, sys

S, C16, V = "P_SCALAR", "P_C16", "P_VECTOR"

# (extension, assembly, kind). Ordered from safe to less safe: if something
# hangs the hart, everything before it has already been reported.
PROBES = [
    # Base ISA letters (cross-checked against misa)
    ("M",        "mul t3, t3, t4", S),
    ("A",        "amoadd.d t3, t4, (a0)", S),
    ("F",        "fadd.s ft0, ft0, ft0", S),
    ("D",        "fadd.d ft0, ft0, ft0", S),
    ("Q",        "fadd.q ft0, ft0, ft0", S),
    ("C",        "c.addi a5, 1", C16),
    # Scalar FP
    ("Zfh",      "fadd.h ft0, ft0, ft0", S),
    ("Zfhmin",   "fcvt.s.h ft0, ft0", S),
    ("Zfbfmin",  "fcvt.bf16.s ft0, ft0", S),
    ("Zfa",      "fli.s ft0, 1.0", S),
    # Atomics
    ("Zacas",    "amocas.d t3, t4, (a0)", S),
    ("Zabha",    "amoadd.b t3, t4, (a0)", S),
    # Bit manipulation and scalar crypto
    ("Zba",      "sh1add t3, t3, t4", S),
    ("Zbb",      "andn t3, t3, t4", S),
    ("Zbb",      "rev8 t3, t3", S),
    ("Zbc",      "clmulr t3, t3, t4", S),
    ("Zbs",      "bset t3, t3, t4", S),
    ("Zbkb",     "pack t3, t3, t4", S),
    ("Zbkb",     "brev8 t3, t3", S),
    ("Zbkc",     "clmul t3, t3, t4", S),
    ("Zbkx",     "xperm8 t3, t3, t4", S),
    ("Zknd",     "aes64ds t3, t3, t4", S),
    ("Zkne",     "aes64es t3, t3, t4", S),
    ("Zknh",     "sha256sig0 t3, t3", S),
    ("Zksed",    "sm4ed t3, t3, t4, 0", S),
    ("Zksh",     "sm3p0 t3, t3", S),
    ("Zkr",      "csrrw t3, seed, zero", S),
    # Misc integer
    ("Zicond",   "czero.eqz t3, t3, t4", S),
    ("Zcb",      "c.zext.b a5", C16),
    ("Zcb",      "c.mul a5, a4", C16),
    ("Zcmop",    "c.mop.1", C16),
    ("Zimop",    "mop.r.0 t3, t3", S),
    ("Zifencei", "fence.i", S),
    ("Zicntr",   "rdcycle t3", S),
    ("Zicntr",   "rdtime t3", S),
    ("Zicntr",   "rdinstret t3", S),
    ("Zihpm",    "csrr t3, hpmcounter3", S),
    # Cache-block operations, on the scratch line only
    ("Zicbom",   "cbo.clean (a0)", S),
    ("Zicbom",   "cbo.flush (a0)", S),
    ("Zicboz",   "cbo.zero (a0)", S),
    # Privileged
    ("S",        "sfence.vma zero, zero", S),
    ("Svinval",  "sinval.vma zero, zero", S),
    ("H",        "hfence.gvma zero, zero", S),
    # SiFive cache control. sf.cdiscard.d.l1 is given an address (the scratch
    # line); with x0 it would discard the whole L1, stack included.
    ("XSiFivecflushdlone",   "sf.cflush.d.l1 zero", S),
    ("XSiFivecdiscarddlone", "sf.cdiscard.d.l1 a0", S),
    # Vector: every probe is tried at each vtype in VTYPES (probe_abi.h). The
    # optional fourth field is the SEW at which success implies the extension
    # (vfwmacc.vv at e32 is plain V, at e16 it needs Zvfh); 0 means any.
    ("V",        "vadd.vv v8, v16, v24", V, 64),
    ("V",        "vfadd.vv v8, v16, v24", V, 64),
    ("Zvfh",     "vfadd.vv v8, v16, v24", V, 16),
    ("Zvfhmin",  "vfwcvt.f.f.v v8, v16", V, 16),
    ("Zvfh",     "vfwmacc.vv v8, v16, v24", V, 16),
    ("Zvfbfmin", "vfwcvtbf16.f.f.v v8, v16", V, 16),
    ("Zvfbfmin", "vfncvtbf16.f.f.w v8, v16", V, 16),
    ("Zvfbfwma", "vfwmaccbf16.vv v8, v16, v24", V, 16),
    ("Zvkb",     "vandn.vv v8, v16, v24", V),
    ("Zvkb",     "vror.vv v8, v16, v24", V),
    ("Zvbb",     "vbrev.v v8, v16", V),
    ("Zvbb",     "vclz.v v8, v16", V),
    ("Zvbb",     "vwsll.vv v8, v16, v24", V),
    ("Zvbc",     "vclmul.vv v8, v16, v24", V, 64),
    ("Zvkg",     "vghsh.vv v8, v16, v24", V, 32),
    ("Zvkned",   "vaesem.vv v8, v16", V, 32),
    ("Zvknha",   "vsha2ms.vv v8, v16, v24", V, 32),
    ("Zvknhb",   "vsha2ms.vv v8, v16, v24", V, 64),
    ("Zvksed",   "vsm4r.vv v8, v16", V, 32),
    ("Zvksh",    "vsm3me.vv v8, v16, v24", V, 32),
    # SiFive Intelligence Extensions as LLVM knows them
    ("XSfvqmaccdod",    "sf.vqmacc.2x8x2 v8, v16, v24", V),
    ("XSfvqmaccdod",    "sf.vqmaccu.2x8x2 v8, v16, v24", V),
    ("XSfvqmaccdod",    "sf.vqmaccus.2x8x2 v8, v16, v24", V),
    ("XSfvqmaccdod",    "sf.vqmaccsu.2x8x2 v8, v16, v24", V),
    ("XSfvqmaccqoq",    "sf.vqmacc.4x8x4 v8, v16, v24", V),
    ("XSfvqmaccqoq",    "sf.vqmaccu.4x8x4 v8, v16, v24", V),
    ("XSfvqmaccqoq",    "sf.vqmaccus.4x8x4 v8, v16, v24", V),
    ("XSfvqmaccqoq",    "sf.vqmaccsu.4x8x4 v8, v16, v24", V),
    ("XSfvfwmaccqqq",   "sf.vfwmacc.4x4x4 v8, v16, v24", V),
    ("XSfvfnrclipxfqf", "sf.vfnrclip.x.f.qf v8, v16, ft0", V),
    ("XSfvfnrclipxfqf", "sf.vfnrclip.xu.f.qf v8, v16, ft0", V),
    # Last: these could plausibly stall rather than trap.
    ("Zawrs",    "wrs.sto", S),
    ("XSfvcp",   "sf.vc.v.vv 3, v8, v16, v24", V),
]

# Fixed instructions the firmware builds its code buffer from.
HELPERS = [
    ("INSN_RET",          "ret"),
    ("INSN_SD_T3_A0",     "sd t3, 0(a0)"),
    ("INSN_VSETVL",       "vsetvl t3, a1, a2"),
    ("INSN_CSRRS_T3_X0",  "csrrs t3, 0, zero"),   # | csr << 20
    ("INSN_CSRRW_T3_A1",  "csrrw t3, 0, a1"),     # | csr << 20: write a1, old value to t3
    ("INSN_CSRRS_T4_X0",  "csrrs t4, 0, zero"),   # | csr << 20: read back
    ("INSN_CSRRW_X0_T3",  "csrrw zero, 0, t3"),   # | csr << 20: restore
    ("INSN_SD_T4_A0",     "sd t4, 0(a0)"),
]

# LLVM 20 has no Q extension for RV64; fadd.q ft0, ft0, ft0 (rm = dyn) by hand.
MANUAL = {"fadd.q ft0, ft0, ft0": (0x03 << 25) | (7 << 12) | 0x53}

MATTR = ",".join("+" + f for f in """m a f d c v zfh zfbfmin zfa zcb zcmop zimop zba zbb zbc zbs zbkb zbkc
    zbkx zknd zkne zknh zksed zksh zkr zicond zicbom zicboz zawrs zacas zabha zicntr zihpm svinval h zvfh
    zvfbfmin zvfbfwma zvbb zvbc zvkg zvkned zvknhb zvksed zvksh xsfvcp xsfvqmaccdod xsfvqmaccqoq
    xsfvfnrclipxfqf xsfvfwmaccqqq xsifivecflushdlone xsifivecdiscarddlone""".split())


def encode(mc, lines, rvc=False):
    todo = [l for l in lines if l not in MANUAL]
    prefix = "" if rvc else ".option norvc\n"   # keep 32-bit encodings 32-bit
    out = subprocess.run([mc, "-triple=riscv64", "-mattr=" + MATTR, "-show-encoding"],
                         input=prefix + "\n".join(todo) + "\n", capture_output=True, text=True)
    if out.returncode:
        sys.exit(out.stderr)
    encs = [re.search(r"encoding: \[([^\]]*)\]", l) for l in out.stdout.splitlines()]
    words = []
    for m in filter(None, encs):
        b = [int(x, 16) for x in m.group(1).split(",")]
        words.append(int.from_bytes(bytes(b), "little"))
    assert len(words) == len(todo), (len(words), len(todo))
    enc = dict(zip(todo, words))
    enc.update(MANUAL)
    return [enc[l] for l in lines]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--llvm-mc", default=shutil.which("llvm-mc") or "/usr/lib/llvm-20/bin/llvm-mc")
    ap.add_argument("-o", default=str(pathlib.Path(__file__).parent / "probes.h"))
    a = ap.parse_args()

    wide = encode(a.llvm_mc, [p[1] for p in PROBES if p[2] != C16])
    narrow = encode(a.llvm_mc, [p[1] for p in PROBES if p[2] == C16], rvc=True)
    probe_words = [(narrow if p[2] == C16 else wide).pop(0) for p in PROBES]
    for (ext, asm, kind, *_), w in zip(PROBES, probe_words):
        assert (w & 3 != 3) == (kind == C16), f"{asm}: 0x{w:x} has the wrong length for {kind}"
    helper_words = encode(a.llvm_mc, [h[1] for h in HELPERS])
    ver = subprocess.run([a.llvm_mc, "--version"], capture_output=True, text=True).stdout
    ver = re.search(r"LLVM version (\S+)", ver).group(1)

    o = ["// SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.",
         "// SPDX-License-Identifier: GPL-2.0-only",
         f"// Generated by gen_probes.py with llvm-mc {ver}. Do not edit; edit gen_probes.py.",
         "//",
         "// X(extension, assembly, encoding, kind, sew): sew is the SEW at which a",
         "// vector probe implies the extension, 0 for any.",
         "#pragma once", ""]
    for (name, _), w in zip(HELPERS, helper_words):
        o.append(f"#define {name:<18} 0x{w:08x}u")
    o += ["", "#define PROBE_LIST(X) \\"]
    for p, w in zip(PROBES, probe_words):
        ext, asm, kind, sew = (p + (0,))[:4]
        o.append(f'    X("{ext}", "{asm}", 0x{w:08x}u, {kind}, {sew}) \\')
    o += ["", ""]
    pathlib.Path(a.o).write_text("\n".join(o))
    print(f"wrote {a.o}: {len(PROBES)} probes")


if __name__ == "__main__":
    main()
