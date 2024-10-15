include := './include'

mbr_source_files := $(wildcard src/mbr/*.nasm)
vbr_source_files := $(wildcard src/vbr/*.nasm)
stage2_source_files := $(wildcard src/stage2/*.nasm)
boottest_source_files := $(wildcard src/miniboot32/*.nasm)

mbr_binary_files := $(patsubst src/mbr/%.nasm, build/%.bin, $(mbr_source_files))
vbr_binary_files := $(patsubst src/vbr/%.nasm, build/%.bin, $(vbr_source_files))
stage2_binary_files := $(patsubst src/stage2/%.nasm, build/%.bin, $(stage2_source_files))
boottest_binary_files := $(patsubst src/miniboot32/%.nasm, build/%.bin, $(boottest_source_files))

build_dir := 'build'

# Get current Git version (tag) and hash
GIT_VERSION := $(shell git describe --tags)
GIT_HASH := $(shell git rev-parse HEAD)
GIT_NASM_DEFINES := -D __GIT_VER__='"$(GIT_VERSION)"' -D __GIT_HASH__='"$(GIT_HASH)"'

iso := '/tmp/disk.img'
isoz := 'disk.img.gz'

qemu_args := -L ./bin/ -bios bios.bin -cpu pentium3 -m 128 -S -s -monitor stdio -nic none
.PHONY: all mbr vbr stage2 boottest clean run run_bochs iso isoz

all: $(iso) $(isoz) $(mbr_binary_files) $(vbr_binary_files) $(stage2_binary_files)
mbr: $(mbr_binary_files)
vbr: $(vbr_binary_files)
stage2: $(stage2_binary_files)
boottest: $(boottest_binary_files)

clean:
	@rm -rvf *.map
	@rm -rvf $(build_dir)/*
	@rm -rvf $(iso)
	@rm -rvf $(isoz)

run: $(iso)
	@sudo qemu-system-i386 $(qemu_args) -hda $(iso)

run_bochs: $(iso)
	@bochs -q

iso: $(iso)
	@file $(iso)

isoz: $(isoz)
	@file $(isoz)

build/%.bin: src/mbr/%.nasm
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $(GIT_NASM_DEFINES) $< -o $@

build/%.bin: src/vbr/%.nasm
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $(GIT_NASM_DEFINES) $< -o $@

build/%.bin: src/stage2/%.nasm
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $(GIT_NASM_DEFINES) $< -o $@

build/%.bin: src/miniboot32/%.nasm
	@mkdir -p $(shell dirname $@)
	@nasm -i$(include) -Wall -f bin $(GIT_NASM_DEFINES) $< -o $@

$(iso): $(mbr_binary_files) $(vbr_binary_files) $(stage2_binary_files) $(boottest_binary_files)
	@echo root access needed to create disk image...
	@sudo scripts/create-disk.sh

$(isoz): $(iso)
	@gzip -9kc $(iso) > $(isoz)
