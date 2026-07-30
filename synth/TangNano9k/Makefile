PROJECT = msx_tangnano9k
DEVICE  = GW1NR-LV9QN88PC6/I5
FAMILY  = GW1N-9C
CST     = tangnano9k.cst

NEXTPNR    = yowasp-nextpnr-himbaechel-gowin
# "yowasp-gowin_pack" is not a real PyPI package; yowasp-nextpnr-himbaechel-gowin
# pulls in Apycula, which provides the actual `gowin_pack` bitstream packer.
GOWIN_PACK = gowin_pack

VHDL_SRCS = \
  ../../src/msx_pack.vhd \
  ../../src/ram/dpram.vhd \
  ../../src/ram/spram.vhd \
  ../../src/shared/fifo.vhd \
  ../../src/cpu/t80_pack.vhd \
  ../../src/cpu/t80_reg.vhd \
  ../../src/cpu/t80_alu.vhd \
  ../../src/cpu/t80_mcode.vhd \
  ../../src/cpu/t80.vhd \
  ../../src/cpu/t80a.vhd \
  ../../src/video/dblscan.vhd \
  ../../src/video/vdp18/vdp18_pack-p.vhd \
  ../../src/video/vdp18/vdp18_paletas_3bit_pack.vhd \
  ../../src/video/vdp18/vdp18_pattern.vhd \
  ../../src/video/vdp18/vdp18_cpuio.vhd \
  ../../src/video/vdp18/vdp18_col_mux.vhd \
  ../../src/video/vdp18/vdp18_palette.vhd \
  ../../src/video/vdp18/vdp18_clk_gen.vhd \
  ../../src/video/vdp18/vdp18_addr_mux.vhd \
  ../../src/video/vdp18/vdp18_hor_vert.vhd \
  ../../src/video/vdp18/vdp18_ctrl.vhd \
  ../../src/video/vdp18/vdp18_sprite.vhd \
  ../../src/video/vdp18/vdp18_core.vhd \
  vga_linebuf.vhd \
  ../../src/audio/dac.vhd \
  ../../src/audio/YM2149.vhd \
  ../../src/peripheral/keymap.vhd \
  ../../src/peripheral/ps2_iobase.vhd \
  ../../src/peripheral/keyboard.vhd \
  ../../src/peripheral/pio.vhd \
  ../../src/peripheral/memoryctl.vhd \
  ../../src/peripheral/exp_slot.vhd \
  ../../src/peripheral/swioports.vhd \
  ../../src/peripheral/spi.vhd \
  ../../src/peripheral/romnextor.vhd \
  ../../src/peripheral/escci/scc_wave.vhd \
  ../../src/peripheral/escci/escci.vhd \
  ../../src/rom/ipl_rom.vhd \
  ../../src/clocks.vhd \
  ../../src/msx.vhd \
  gowin_rpll.vhd \
  spi_master.vhd \
  flash_reader.vhd \
  flash_rom_xip.vhd \
  top_tangnano9k.vhd

all: $(PROJECT).fs

# 1. VHDL -> Verilog 넷리스트 변환
msx_vhdl_netlist.v: $(VHDL_SRCS)
	ghdl -a --ieee=synopsys -fexplicit -P=../../src/ --work=work $(VHDL_SRCS)
	ghdl --synth --ieee=synopsys -fexplicit --out=verilog -P=../../src/ --work=work top_tangnano9k > msx_vhdl_netlist.v

# 2. Yosys Gowin Synthesis
$(PROJECT).json: msx_vhdl_netlist.v dvi_tx.v hdmi_rpll.v hdmi_clkdiv.v synth.ys
	yosys -s synth.ys

# 3. Place & Route (YoWASP Himbaechel 옵션 지정)
$(PROJECT)_pnr.json: $(PROJECT).json $(CST)
	$(NEXTPNR) --json $(PROJECT).json \
	           --write $(PROJECT)_pnr.json \
	           --device $(DEVICE) \
	           --vopt family=$(FAMILY) \
	           --vopt cst=$(CST)

# 4. Bitstream Generation
# --mspi_as_gpio: releases pins 59/60/61/62 (flash_clk/ncs/do/di) from the
# GW1NR-9C's dedicated MSPI hard config-boot-flash controller so our own
# flash_reader.vhd logic actually drives/reads those physical pins, instead
# of silently losing the pin ownership fight with that hard block (no error
# is raised by the toolchain either way - this only shows up as the SPI
# flash reads never reaching the real chip at runtime).
$(PROJECT).fs: $(PROJECT)_pnr.json
	$(GOWIN_PACK) -d $(FAMILY) --mspi_as_gpio -o $(PROJECT).fs $(PROJECT)_pnr.json

load: $(PROJECT).fs
	openFPGALoader -b tangnano9k $(PROJECT).fs

clean:
	rm -f *.json *.fs *.cf work-obj*.cf msx_vhdl_netlist.v

# =========================================================================
# Standalone PSRAM built-in-self-test (BIST) - see psram_bist_top.vhd.
# Separate GHDL work library (--work=bist) so this doesn't clobber/conflict
# with the main design's own `work-obj93.cf`.
# =========================================================================
BIST_VHDL_SRCS = gowin_rpll.vhd uart_tx.vhd psram_bist_top.vhd

psram_bist_netlist.v: $(BIST_VHDL_SRCS)
	ghdl -a --ieee=synopsys -fexplicit --work=bist $(BIST_VHDL_SRCS)
	ghdl --synth --ieee=synopsys -fexplicit --out=verilog --work=bist psram_bist_top > psram_bist_netlist.v

psram_bist.json: psram_bist_netlist.v psram_controller.v psram_ram_bridge.v synth_bist.ys
	yosys -s synth_bist.ys

SEED ?= 1
psram_bist_pnr.json: psram_bist.json psram_bist.cst
	$(NEXTPNR) --json psram_bist.json \
	           --write psram_bist_pnr.json \
	           --device $(DEVICE) \
	           --vopt family=$(FAMILY) \
	           --vopt cst=psram_bist.cst \
	           --seed $(SEED)

psram_bist.fs: psram_bist_pnr.json
	$(GOWIN_PACK) -d $(FAMILY) --mspi_as_gpio -o psram_bist.fs psram_bist_pnr.json

bist: psram_bist.fs

bist-load: psram_bist.fs
	openFPGALoader -b tangnano9k psram_bist.fs

bist-clean:
	rm -f psram_bist*.json psram_bist.fs psram_bist_netlist.v bist-obj*.cf

.PHONY: all load flash clean bist bist-load bist-clean
