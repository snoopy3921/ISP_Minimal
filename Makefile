# Copyright (c) 2024-2025 Arm Limited. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

projname=ISP_minimal
default_args=--packs --update-rte --jobs 8 --rebuild --context .Debug
cbuild=cbuild
model=FVP_Corstone_SSE-320
model_args=-C mps4_board.uart0.out_file=- \
	-C mps4_board.isp_c55_camera.image_file=ISP_minimal/test.frm \
	-C mps4_board.isp_c55_capture_ds.do_capture=0 \
	-C mps4_board.isp_c55_capture_fr.do_capture=0 \
	-C mps4_board.telnetterminal0.mode=raw \
	--simlimit 10

all:
	$(MAKE) clean armclang
	$(MAKE) clean gcc

run-all:
	parallel $(model) $(model_args) ::: bins/*

armclang:
	$(cbuild) $(projname).csolution.yml $(default_args) --toolchain AC6
	mkdir -p bins
	mkdir -p bins/fvp
	cp build/SSE-320-FVP/AC6/Debug/$(projname)/outdir/$(projname).axf bins/$(projname).axf
	fromelf --bin --output bins/$(projname)_armclang.bin bins/$(projname).axf

gcc:
	$(cbuild) $(projname).csolution.yml $(default_args) --toolchain GCC
	mkdir -p bins
	mkdir -p bins/fvp
	cp build/SSE-320-FVP/GCC/Debug/$(projname)/outdir/$(projname).elf bins/$(projname).elf
	arm-none-eabi-objcopy -O binary bins/$(projname).elf bins/$(projname)_gcc.bin

clean:
	rm -rf $(projname)/$(projname)*.[RD]*+*-*.*
	rm -rf *.cbuild-idx.yml

clean-build:
	rm -rf build bins

run-armclang:
	$(model) $(model_args) build/SSE-320-FVP/AC6/Debug/$(projname)/outdir/$(projname).axf

run-gcc:
	$(model) $(model_args) build/SSE-320-FVP/GCC/Debug/$(projname)/outdir/$(projname).elf
