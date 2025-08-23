#!/usr/bin/env bash
set -euo pipefail

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

# paths to bootcode
mbr_file=build/mbr.bin
vbr_file=build/vbr.bin
stage2_file=build/stage2.bin
boottest_file=build/BOOTi686.bin

# Disk creation options
disk_img=/tmp/disk.img
disk_img_final=build/disk.img.gz
part_img=/tmp/part.img

# $disk_sector_size * $disk_size = total bytes, default is 256MiB
disk_sectors=(524288 * 2)
disk_sector_size=512
part_start=2048
part_sectors=$((disk_sectors - part_start))

#
# Build sanity section
#

echo "*** Performing pre-build sanity checks ***"

# Tools needed:
#   sfdisk dosfstools (mkfs.fat), mtools (mcopy,mmd), gzip, dd, truncate, awk
for t in mcopy mmd gzip dd truncate awk; do
  command -v "$t" >/dev/null || { echo "Missing tool: $t" >&2; exit 1; }
done

# these are normally in the sbin paths so I've had issues detecting them with command -v
SF=$(command -v sfdisk || echo /usr/sbin/sfdisk)
MKFS=$(command -v mkfs.fat || echo /usr/sbin/mkfs.fat)

[[ -x "$SF" ]] || { echo "sfdisk not found"; exit 1; }
[[ -x "$MKFS" ]] || { echo "mkfs.fat not found"; exit 1; }

# check that required build files exist
for f in "$mbr_file" "$vbr_file" "$stage2_file" "$boottest_file"; do
  [[ -f "$f" ]] || { echo "missing $f" >&2; exit 1; }
done

cat > /tmp/pt.sfdisk <<EOF
label: dos
unit: sectors
sector-size: $disk_sector_size
label-id: 0xa0b0c0d0

start=$part_start, size=$((disk_sectors - part_start)), type=c, bootable
EOF

#
# Create disk images
#

echo "[1/7] Create disk.img and part.img"

if ! [ -e "$disk_img" ]; then
    # create raw disk image
    if ! truncate -s $((disk_sectors * disk_sector_size)) "$disk_img"; then
        echo "Failed creating blank disk image." >&2
        exit 1
    fi
    sync
else
    echo "Removing old disk image..."
    rm -rfv "$disk_img"
    if ! truncate -s $((disk_sectors * disk_sector_size)) "$disk_img"; then
        echo "Failed creating blank disk image." >&2
        exit 1
    fi
    sync
fi

if ! [ -e "$part_img" ]; then
    # create raw partition disk image
    if ! truncate -s $((part_sectors * disk_sector_size)) "$part_img"; then
        echo "Failed creating blank partition image." >&2
        exit 1
    fi
    sync
else
    echo "Removing old (partition) disk image..."
    rm -rfv "$part_img"
    if ! truncate -s $((part_sectors * disk_sector_size)) "$part_img"; then
        echo "Failed creating blank partition image." >&2
        exit 1
    fi
    sync
fi


if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        echo "[2/7] Write DOS partition table (single FAT32 LBA @ 2048)"
        "$SF" --no-reread "$disk_img" < /tmp/pt.sfdisk

        echo "[3/7] Make FAT32 filesystem in partition image"
        "$MKFS" -v -F32 -s 1 -n 'STEVIAFS' "$part_img"

        echo "[4/7] Patch VBR inside partition image (preserve BPB)"

        # copy jmp short entry; nop
        dd if=$vbr_file of="$part_img" bs=1 count=3 conv=notrunc
        # copy bootcode
        dd if=$vbr_file of="$part_img" bs=1 seek=90 skip=90 count=420 conv=notrunc
        # copy signature (should be 0xAA55)
        dd if=$vbr_file of="$part_img" bs=1 seek=510 skip=510 count=2 conv=notrunc

        # copy backup VBR within the created partition image
        # Linux dosfstools will complain (read: not work) unless this is done it seems
        # HACK: sector 6 is the **default** location of the BPB_BkBootSec, it **can** be different.
        dd if="$part_img" of="$part_img" bs=$disk_sector_size count=1 seek=6 conv=notrunc

        echo "[5/7] Copy boot payload to FAT32 filesystem using mtools as BOOT.BIN"
        mcopy -i "$part_img" "$boottest_file" ::/BOOT.BIN

        echo "[6/7] Patch MBR and install stage2 loader to disk image"
        # patch MBR+signature while preserving partition table
        dd if="$mbr_file" of="$disk_img" bs=1 count=440 conv=notrunc
        dd if="$mbr_file" of="$disk_img" bs=1 seek=510 skip=510 count=2 conv=notrunc

        # copy stage2 to absolute LBA 1
        dd if="$stage2_file" of="$disk_img" bs=$disk_sector_size seek=1 conv=notrunc

        echo "[7/7] Assembling final disk image"

        # place partition at it's place in the disk image
        dd if="$part_img" of="$disk_img" bs=$disk_sector_size seek=$part_start conv=notrunc
        gzip -9c "$disk_img" > "$disk_img_final"

# requires util-linux from homebrew
elif [[ "$OSTYPE" == "darwin"* ]]; then
        echo "[WIP]"
else
        # Unknown.
        echo "Unknown OS type! Supported build hosts systems are GNU/Linux (WSL) and macOS."
        exit 1
fi