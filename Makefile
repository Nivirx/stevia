iso := 'disk.img'

include := './include'

mbr_source_files := $(wildcard src/mbr/*.s)
vbr_source_files := $(wildcard src/vbr/*.s)
stage2_source_files := $(wildcard src/stage2/*.s)
boottest_source_files := $(wildcard src/miniboot32/*.s)

mbr_binary_files := $(patsubst src/mbr/%.s, build/%.bin, $(mbr_source_files))
vbr_binary_files := $(patsubst src/vbr/%.s, build/%.bin, $(vbr_source_files))
stage2_binary_files := $(patsubst src/stage2/%.s, build/%.bin, $(stage2_source_files))
boottest_binary_files := $(patsubst src/miniboot32/%.s, build/%.bin, $(boottest_source_files))

build_dir := 'build'

qemu_args := -L ./bin/ -bios bios.bin -cpu pentium3 -m 64 -S -s -monitor stdio -nic none
.PHONY: all mbr vbr stage2 boottest clean run run_bochs iso

all: $(iso) $(mbr_binary_files) $(vbr_binary_files) $(stage2_binary_files)
mbr: $(mbr_binary_files)
vbr: $(vbr_binary_files)
stage2: $(stage2_binary_files)
boottest: $(boottest_binary_files)

clean:
	@rm -rvf $(build_dir)/*
	@rm -rvf $(iso)

run: $(iso)
	@sudo qemu-system-i386 $(qemu_args) -hda $(iso)

run_bochs: $(iso)
	@bochs -q

iso: $(iso)
	@file disk.img

build/%.bin: src/mbr/%.s
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $< -o $@

build/%.bin: src/vbr/%.s
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $< -o $@

build/%.bin: src/stage2/%.s
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $< -o $@

build/%.bin: src/miniboot32/%.s
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $< -o $@

$(iso): $(mbr_binary_files) $(vbr_binary_files) $(stage2_binary_files) $(boottest_binary_files)
	@echo root access needed to create disk image...
	@sudo scripts/create-disk.sh

