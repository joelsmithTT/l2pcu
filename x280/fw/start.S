# SPDX-FileCopyrightText: © 2026 Tenstorrent Inc.
# SPDX-License-Identifier: GPL-2.0-only
#
# start.S - reset entry for dma_fw. All four harts arrive here with the
# reset vector pointing at _start. Hart 0 sets up a trap vector, a stack and
# calls main_hart0(); the others park in wfi.

    .section .text.start, "ax"
    .globl _start
_start:
    csrr    t0, mhartid
    bnez    t0, park
    la      t0, trap_vec
    csrw    mtvec, t0
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
    csrr    a0, mcause
    csrr    a1, mepc
    csrr    a2, mtval
    la      sp, __stack_top
    call    trap_handler_c
    j       park
