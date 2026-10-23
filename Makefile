# Top-level build. `make` builds everything:
#   host/l2cpu_dma_test    host-driven DMA library tester (needs only g++)
#   x280/fw/dma_fw.bin     X280 firmware (needs a RISC-V cross compiler)
#   x280/x280_dma_host     boots dma_fw.bin and has the X280 drive the DMA
#   isa/fw/probe_fw.bin    X280 firmware that probes for ISA extensions
#   isa/isa_probe          boots probe_fw.bin and reports what the X280 implements

CXX      ?= g++
CXXFLAGS ?= -std=c++17 -O2 -g -Wall -Wextra

all: host x280 isa

host: host/l2cpu_dma_test
x280: x280/x280_dma_host fw
isa: isa/isa_probe isa-fw

host/l2cpu_dma_test: host/l2cpu_dma_test.cpp host/l2cpu_dma.hpp
	$(CXX) $(CXXFLAGS) -o $@ $<

x280/x280_dma_host: x280/x280_dma_host.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<

isa/isa_probe: isa/isa_probe.cpp isa/probe_abi.h isa/probes.h host/l2cpu_dma.hpp
	$(CXX) $(CXXFLAGS) -o $@ $<

fw:
	$(MAKE) -C x280/fw

isa-fw:
	$(MAKE) -C isa/fw

clean:
	rm -f host/l2cpu_dma_test x280/x280_dma_host isa/isa_probe
	$(MAKE) -C x280/fw clean
	$(MAKE) -C isa/fw clean

.PHONY: all host x280 isa fw isa-fw clean
