# Builds dma_fw.bin, the bare-metal X280 firmware that x280_dma_host loads.
# Any RISC-V GCC that can target rv64imac/lp64 works; override CROSS to pick
# another prefix, e.g. `make CROSS=riscv64-linux-gnu-`.

CROSS   ?= riscv64-unknown-elf-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump

CFLAGS := -march=rv64imac -mabi=lp64 -mcmodel=medany -O2 -g -Wall -Wextra \
          -ffreestanding -fno-builtin -nostdlib -fno-pic -ffunction-sections -fdata-sections -fno-common
LDFLAGS := -T link.ld -nostartfiles -nostdlib -Wl,--gc-sections -Wl,--no-relax

all: dma_fw.bin dma_fw.lst

dma_fw.elf: start.S dma_fw.c link.ld
	@command -v $(CC) >/dev/null || { echo "$(CC) not found: install a RISC-V cross compiler" \
	    "(Ubuntu: apt install gcc-riscv64-unknown-elf) or set CROSS=<prefix>"; exit 1; }
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ start.S dma_fw.c -Wl,-Map=dma_fw.map

dma_fw.bin: dma_fw.elf
	$(OBJCOPY) -O binary $< $@
	@echo "dma_fw.bin: $$(wc -c < $@) bytes, sha256 $$(sha256sum $@ | cut -c1-16)"

dma_fw.lst: dma_fw.elf
	$(OBJDUMP) -d $< > $@

clean:
	rm -f dma_fw.elf dma_fw.bin dma_fw.lst dma_fw.map

.PHONY: all clean
