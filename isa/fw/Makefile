# Builds probe_fw.bin, the bare-metal X280 firmware that isa_probe loads.
# Any RISC-V GCC that can target rv64imac/lp64 works; override CROSS to pick
# another prefix, e.g. `make CROSS=riscv64-linux-gnu-`.

CROSS   ?= riscv64-unknown-elf-
CC      := $(CROSS)gcc
OBJCOPY := $(CROSS)objcopy
OBJDUMP := $(CROSS)objdump

CFLAGS := -march=rv64imac -mabi=lp64 -mcmodel=medany -O2 -g -Wall -Wextra \
          -ffreestanding -fno-builtin -nostdlib -fno-pic -ffunction-sections -fdata-sections -fno-common
LDFLAGS := -T link.ld -nostartfiles -nostdlib -Wl,--gc-sections -Wl,--no-relax

all: probe_fw.bin probe_fw.lst

probe_fw.elf: start.S probe_fw.c link.ld ../probe_abi.h ../probes.h
	@command -v $(CC) >/dev/null || { echo "$(CC) not found: install a RISC-V cross compiler" \
	    "(Ubuntu: apt install gcc-riscv64-unknown-elf) or set CROSS=<prefix>"; exit 1; }
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ start.S probe_fw.c -Wl,-Map=probe_fw.map

probe_fw.bin: probe_fw.elf
	$(OBJCOPY) -O binary $< $@
	@echo "probe_fw.bin: $$(wc -c < $@) bytes, sha256 $$(sha256sum $@ | cut -c1-16)"

probe_fw.lst: probe_fw.elf
	$(OBJDUMP) -d $< > $@

clean:
	rm -f probe_fw.elf probe_fw.bin probe_fw.lst probe_fw.map

.PHONY: all clean
