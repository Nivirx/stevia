#!/usr/bin/env bash

# Copyright (c) 2023 Elaina Claus
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

if ! [ $(id -u) = 0 ]; then
   echo "Script must be run as root!"
   exit 1
fi

mbr_file=build/mbr.bin
vbr_file=build/vbr.bin
stage2_file=build/stage2.bin
boottest_file=build/BOOT_386.bin

mount_point=/tmp/stevia_disk

if ! [ -e disk.img ]; then
    # create raw disk image with 128MiB of space
    dd if=/dev/zero of=disk.img bs=512 count=262144
    sync
else
    echo "Removing old disk image..."
    rm -rfv disk.img
    dd if=/dev/zero of=disk.img bs=512 count=262144
    sync
fi

if [[ "$OSTYPE" == "linux-gnu"* ]]; then
        if [ -e $mbr_file ] && [ -e $vbr_file ]; then
        # get next loop device and mount it
        ld=$(losetup -f)
        losetup -P -b 512 $ld disk.img

        # create a DOS disk, with 1 FAT32 partition that is bootable, part1 starts at sector 2048
        sfdisk $ld < scripts/loop_setup.sfdisk

        # partprobe the image
        partprobe $ld

        # get first partition
        firstpart=$(lsblk -ilp -o NAME $ld | tr '\n' ' ' | awk '{print $3}')
        mkfs.vfat -v -F32 -S 512 $firstpart

        # copy MBR while preserving partition table
        dd if=$mbr_file of=$ld bs=1 count=440
        # copy MBR 0xAA55
        dd if=$mbr_file of=$ld bs=1 seek=510 skip=510 count=2

        # copy VBR to partition 1 while preserving partition information
        # copy jmp short entry; nop
        dd if=$vbr_file of=$firstpart bs=1 count=3
        # copy bootcode
        dd if=$vbr_file of=$firstpart bs=1 seek=90 skip=90 count=420
        # copy 0xAA55
        dd if=$vbr_file of=$firstpart bs=1 seek=510 skip=510 count=2

        #stage2 to sectors 1-64
        dd if=$stage2_file of=$ld bs=512 seek=1

        # copy boot32 boot test file to disk image
        if ! [ -e $mount_point ]; then
            mkdir $mount_point
        fi
        mount $firstpart $mount_point

        if [ -e $boottest_file ]; then
            cp -v $boottest_file $mount_point
        else
            echo "unable to find boot32.bin!"
            exit 3
        fi

        # detach loop device
        umount $mount_point
        sync
        losetup -d $ld

        # chown to the real user to prevent issues with reading/writing the file later
        SUDOUSER=$(logname)
        chown --from=root:root ${SUDOUSER}:$(id $SUDOUSER -g) disk.img

    else
        echo "unable to find MBR/VBR binaries!"
        exit 2
    fi
# requires util-linux from homebrew
elif [[ "$OSTYPE" == "darwin"* ]]; then
        sfdisk_path="/usr/local/opt/util-linux/sbin/sfdisk"
        if [ -e $mbr_file ] && [ -e $vbr_file ]; then
        # use hdiutil to attach our empty disk image
        ld_path_raw=$(hdiutil attach -readwrite -imagekey diskimage-class=CRawDiskImage -nomount -blocksize 512 -noverify disk.img)
        ld=$(echo $ld_path_raw | sed s:/dev/::)
        ld_path="/dev/$ld"

        # create a DOS disk, with 1 FAT32 partition
        if ! [ -e $sfdisk_path ]; then
            echo "sfdisk utility was not found...We cannot use diskutil to make disks due to the fact that it only makes Hybrid MBR's & GPT disks...blame Apple"
            exit 4
        else
            $sfdisk_path $ld_path < scripts/loop_setup.sfdisk
        fi

        # give stuff a chance to settle, macOS has problems here
        sync
        sleep .5

        # reattch the raw image file since macOS doesn't have a partprobe...
        # this is the only way I know to get macOS to reprobe the disk
        hdiutil eject $ld
        unset ld_path
        unset ld
        unset ld_path_raw

        ld_path_raw=$(hdiutil attach -readwrite -imagekey diskimage-class=CRawDiskImage -nomount -blocksize 512 -noverify disk.img)
        ld=$(echo $ld_path_raw | grep "FDisk_partition_scheme" | awk '{print $1}' | sed s:/dev/::)
        ld_path="/dev/$ld"

        if ! [ -b /dev/$ld ]; then
            echo "Unable to remount disk! exitting before I do some damage!"
            exit 5
        fi

        # get first partition and format as FAT32
        firstpart=$(diskutil list $ld | grep 1: | awk '{print $6}')
        firstpart_direct="r$firstpart"

        newfs_msdos -F 32 /dev/$firstpart_direct
        sync
        
        # copy MBR while preserving partition table
        dd if=$mbr_file of=$ld_path bs=1 count=440 conv=sync
        # copy MBR 0xAA55
        dd if=$mbr_file of=$ld_path bs=1 seek=510 skip=510 count=2 conv=sync

        # copy VBR to partition 1 while preserving partition information
        # copy jmp short entry; nop
        dd if=$vbr_file of=/dev/$firstpart bs=1 count=3 conv=sync
        # copy bootcode
        dd if=$vbr_file of=/dev/$firstpart bs=1 seek=90 skip=90 count=420 conv=sync
        # copy 0xAA55
        dd if=$vbr_file of=/dev/$firstpart bs=1 seek=510 skip=510 count=2 conv=sync

        #stage2 to sectors 1-64
        dd if=$stage2_file of=$ld_path bs=512 seek=1 conv=sync

        #sync pending dd stuff
        sync

        # copy boot32 boot test file to disk image
        if ! [ -e $mount_point ]; then
            mkdir $mount_point
        else
            echo "$mount_point exists! clearing contents..."
            rm -rfv $mount_point
            mkdir $mount_point
        fi
        mount -t msdos /dev/$firstpart $mount_point

        if [ -e $boottest_file ]; then
            cp -v $boottest_file $mount_point
        else
            echo "unable to find boot32.bin!"
            exit 3
        fi

        # detach loop device
        sync
        umount /dev/$firstpart
        hdiutil eject $ld

        # chown to the real user to prevent issues with reading/writing the file later
        SUDOUSER=$(logname)
        chown ${SUDOUSER}:staff disk.img

    else
        echo "unable to find MBR/VBR binaries!"
        exit 2
    fi
else
        # Unknown.
        echo "Unknown OS type! Supported build hosts systems are GNU/Linux (WSL) and macOS."
        exit 1
fi