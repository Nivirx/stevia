# Copyright (C) 2025 Elaina Claus
# 
#     This program is free software: you can redistribute it and/or modify
#     it under the terms of the GNU General Public License as published by
#     the Free Software Foundation, either version 3 of the License, or
#     (at your option) any later version.
# 
#     This program is distributed in the hope that it will be useful,
#     but WITHOUT ANY WARRANTY; without even the implied warranty of
#     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#     GNU General Public License for more details.
# 
#     You should have received a copy of the GNU General Public License
#     along with this program.  If not, see <https://www.gnu.org/licenses/>.

INCDIR := include
BUILD_DIR := build
IMG := $(BUILD_DIR)/disk.img
IMGZ := $(BUILD_DIR)/disk.img.gz
DEP_DIR     := $(BUILD_DIR)/deps

# Get current Git version (tag) and hash
GIT_VERSION := $(shell git describe --tags --always --dirty 2>/dev/null || echo unknown)
GIT_HASH := $(shell git rev-parse HEAD 2>/dev/null || echo unknown)
GIT_NASM_DEFINES := -D __GIT_VER__='"$(GIT_VERSION)"' -D __GIT_HASH__='"$(GIT_HASH)"'

NASMFLAGS   := -Wall -f bin -i$(INCDIR)/ $(GIT_NASM_DEFINES)

QEMU        ?= qemu-system-i386
QEMU_ARGS := \
    -M pc-i440fx-8.2,accel=tcg \
    -cpu pentium3 \
    -m 64M \
    -rtc base=localtime \
    -boot c \
    -vga std \
    -display gtk,gl=off \
    -device e1000,netdev=n0,mac=52:54:00:12:34:56 \
    -netdev user,id=n0 \
    -device piix3-usb-uhci \
    -parallel null \
    -serial null -serial none -serial none -serial none \
    -chardev file,id=dbg,path=bochs-e9.log,append=on \
    -device isa-debugcon,iobase=0xe9,chardev=dbg \
    -msg timestamp=on

MBR_SRCS 	  := $(wildcard src/mbr/*.nasm)
VBR_SRCS 	  := $(wildcard src/vbr/*.nasm)
STAGE2_SRCS   := $(wildcard src/stage2/*.nasm)
BOOTTEST_SRCS := $(wildcard src/miniboot32/*.nasm)

MBR_BINS 	  := $(patsubst src/mbr/%.nasm, $(BUILD_DIR)/mbr/%.bin, $(MBR_SRCS))
VBR_BINS 	  := $(patsubst src/vbr/%.nasm, $(BUILD_DIR)/vbr/%.bin, $(VBR_SRCS))
STAGE2_BINS   := $(patsubst src/stage2/%.nasm, $(BUILD_DIR)/stage2/%.bin, $(STAGE2_SRCS))
BOOTTEST_BINS := $(patsubst src/miniboot32/%.nasm, $(BUILD_DIR)/miniboot32/%.bin, $(BOOTTEST_SRCS))

ALL_BINS	  := $(MBR_BINS) $(VBR_BINS) $(STAGE2_BINS) $(BOOTTEST_BINS)

.DEFAULT_GOAL := all
.DELETE_ON_ERROR:

.PHONY: all mbr vbr stage2 boottest clean run run_qemu run_bochs img imgz help
all: $(IMG) $(IMGZ) $(ALL_BINS)
mbr: $(MBR_BINS)
vbr: $(VBR_BINS)
stage2: $(STAGE2_BINS)
boottest: $(BOOTTEST_BINS)

# Build Rules

$(BUILD_DIR)/mbr/%.bin: src/mbr/%.nasm | $(DEP_DIR)
	@mkdir -p $(@D)
	@nasm $(NASMFLAGS) -MD $(DEP_DIR)/$*.d -MT $@ $< -o $@

$(BUILD_DIR)/vbr/%.bin: src/vbr/%.nasm | $(DEP_DIR)
	@mkdir -p $(@D)
	@nasm $(NASMFLAGS) -MD $(DEP_DIR)/$*.d -MT $@ $< -o $@

$(BUILD_DIR)/stage2/%.bin: src/stage2/%.nasm | $(DEP_DIR)
	@mkdir -p $(@D)
	@nasm $(NASMFLAGS) -MD $(DEP_DIR)/$*.d -MT $@ $< -o $@

$(BUILD_DIR)/miniboot32/%.bin: src/miniboot32/%.nasm | $(DEP_DIR)
	@mkdir -p $(@D)
	@nasm $(NASMFLAGS) -MD $(DEP_DIR)/$*.d -MT $@ $< -o $@


$(DEP_DIR):
	@mkdir -p $@

# Disk image's

$(IMG): $(ALL_BINS) | scripts/create-disk.sh
	@scripts/create-disk.sh

$(IMGZ): $(IMG)
	@gzip -9kc $(IMG) > $(IMGZ)

img: $(IMG)
	@file $(IMG)

imgz: $(IMGZ)
	@file $(IMGZ)

# Helpers

run: $(IMG)
	@$(QEMU) $(QEMU_ARGS) -drive file=$(IMG),if=ide,index=0,media=disk,format=raw \

run_bochs: $(IMG)
	@bochs -q

clean:
	@$(RM) -rv $(BUILD_DIR)/

help:
	@awk '/^[A-Za-z0-9_%-]+:/{print $$1}' $(MAKEFILE_LIST) | sed 's/:$$//' | sort -u

# Include header deps
-include $(wildcard $(DEP_DIR)/*.d)