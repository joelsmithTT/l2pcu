/* probe_fw is linked at the cached GDDR alias of L2CPU 0's load offset but is
 * position independent (medany, no absolute pointers), so the host may load
 * it at any 4 KiB aligned address. Image + stack must stay under 64 KiB: the
 * results sit at load + RESULTS_OFFSET (0x10000, through the uncached alias). */
OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY { IMG (rwx) : ORIGIN = 0x400120000000, LENGTH = 64K }

__stack_size = 8K;

SECTIONS
{
    .text : ALIGN(8) { *(.text.start) *(.text .text.*) . = ALIGN(8); } > IMG
    .rodata : ALIGN(8) { *(.rodata .rodata.* .srodata .srodata.*) . = ALIGN(8); } > IMG
    .data : ALIGN(8) { *(.data .data.* .sdata .sdata.*) . = ALIGN(8); } > IMG
    .bss : ALIGN(8) { __bss_start = .; *(.bss .bss.* .sbss .sbss.* COMMON) . = ALIGN(8); __bss_end = .; } > IMG
    .stack (NOLOAD) : ALIGN(16) { . = . + __stack_size; __stack_top = .; } > IMG
    /DISCARD/ : { *(.note.GNU-stack) *(.comment) *(.eh_frame) *(.eh_frame_hdr) *(.riscv.attributes) }
}
