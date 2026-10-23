# SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
# SPDX-License-Identifier: GPL-2.0-only
#
# start.S - reset entry and trap handler for probe_fw. Hart 0 sets up a stack
# and the trap vector and calls main_hart0(); harts 1..3 park in wfi.
#
# The trap handler is what makes probing work. While trap_save.armed is set,
# any trap is counted, its mcause recorded, and execution resumes after the
# faulting instruction (2 or 4 bytes, read from mepc). It touches only t0..t2,
# which it saves through mscratch, so the probed code sees no difference.
# Outside a probe a trap is fatal: trap_handler_c records it and parks.

    .equ SAVE_T1,    0
    .equ SAVE_T2,    8
    .equ SAVE_ARMED, 16
    .equ SAVE_CAUSE, 24
    .equ SAVE_COUNT, 32

    .section .text.start, "ax"
    .globl _start
_start:
    csrr    t0, mhartid
    bnez    t0, park
    la      t0, trap_vec
    csrw    mtvec, t0
    la      t0, trap_save
    csrw    mscratch, t0
    la      sp, __stack_top
    # zero .bss
    la      t0, __bss_start
    la      t1, __bss_end
1:  bgeu    t0, t1, 2f
    sd      zero, 0(t0)
    addi    t0, t0, 8
    j       1b
2:  call    main_hart0
park:
    wfi
    j       park

    .align 4
trap_vec:
    csrrw   t0, mscratch, t0            # t0 = &trap_save, mscratch = caller's t0
    sd      t1, SAVE_T1(t0)
    sd      t2, SAVE_T2(t0)
    ld      t1, SAVE_ARMED(t0)
    beqz    t1, fatal
    ld      t1, SAVE_COUNT(t0)
    bnez    t1, 3f                      # keep the first cause
    csrr    t2, mcause
    sd      t2, SAVE_CAUSE(t0)
3:  addi    t1, t1, 1
    sd      t1, SAVE_COUNT(t0)
    csrr    t1, mepc                    # skip the instruction: 4 bytes if bits [1:0] are 11, else 2
    lhu     t2, 0(t1)
    andi    t2, t2, 3
    addi    t1, t1, 2
    xori    t2, t2, 3
    bnez    t2, 4f
    addi    t1, t1, 2
4:  csrw    mepc, t1
    ld      t1, SAVE_T1(t0)
    ld      t2, SAVE_T2(t0)
    csrrw   t0, mscratch, t0
    mret

fatal:
    csrr    a0, mcause
    csrr    a1, mepc
    csrr    a2, mtval
    la      sp, __stack_top
    call    trap_handler_c
    j       park
