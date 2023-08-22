# stevia

This is a hobby project that aims to create a simple and lightweight operating system kernel. It is written in Assembly and C, and uses the GNU toolchain and Bochs emulator. The project features (or is hoping to feature):

- A boot loader that loads the kernel from a floppy disk image
- A kernel that implements basic functions, such as printing messages, handling interrupts, and managing memory
- A shell that allows user input and execution of commands
- A simple text editor that can create and edit files
- A calculator that can perform arithmetic operations
- (big maybe) micropython or some other language, cross compiling, etc...

## Why?

I enjoy bare metal programing and I have an old Dell system with a Pentium 3, I did this mostly to learn how to write a legacy style bootloader (I am also kinda working on a UEFI bootloader/utility as well!)

## Installation

To build and run this project, you need to have the following tools installed:

- A host system running Linux, macOS (should work 🤷‍♀️), or Windows 11 with WSL2/WSLg
- GNU Binutils
- GNU GCC
- GNU Make
- NASM
- Bochs (for testing, project might run on other virtuization/emulation platforms, I target the Pentium 3 Era with this project)

To build the project, run `make` in the root directory. This will generate a floppy disk image named `stevia.img` that contains the boot loader and the kernel.

To run the project, run `bochs -f bochsrc.txt` in the root directory. This will launch the Bochs emulator and load the floppy disk image. You should see the boot loader message, followed by the kernel message, and then the shell prompt.

To exit the emulator, press `Ctrl+C` in the terminal where you launched Bochs.

To run the project on real hardware I use a SD card to IDE interface for my test system which is a ~700Mhz P3 with 256 MB of ram. You will need to write the 'stevia.img' to the disk with 'dd'. On real hardware you may encounter issues that are not accounted for since most of the development happens on bochs, please report any issues you encounter.

## License

This project is licensed under the MIT License. See the [LICENSE.md](^2^) file for details.

### mbr/mbr.s

This file contains the code for a Master Boot Record (MBR) that attempts to find an active partition to load. The CPU is assumed to be in 16-bit real mode at this point and the bios has loaded the 1st sector of the disk (512 bytes) to 0x0000:0x7C00. First the mbr code relocates from 0x7C00 to 0x0600 and then jumps to the entry point. the loader attempts to load the first active partition it can find and loads the vbr to 0x7c00 (512 bytes) and then jumps to 0x0000:0x7C00. It is written in Assembly and uses BIOS interrupts to read sectors from disk.[^note]

### vbr/vbr.s

This file contains the code for a Volume Boot Record (VBR) that loads the stage2 loader code from the booted disk. It reads disk sectors 1-63 into memory to STAGE2_ENTRY(defaults to 0x7E00 in config.inc) this currently allows for stevia to load up to 32KiB of code to perform bootloader duties. It is written in Assembly and uses BIOS interrupts to read sectors from disk.[^note]

### stage2/stage2.s

This file contains the code for a second-stage boot loader that loads additional modules from disk, performs some system sanity checks, and provides a simple boot interface for options/configuration. It is written in Assembly and uses BIOS interrupts to read sectors from disk.[^note]

### miniboot32/BOOT_386.s

This file contains the code for a 32-bit binary that can be loaded and ran with stevia as a mini demo. It is written in Assembly.[^note]

### scripts/create-disk.sh

This script creates a disk image file for stevia. The disk image file can be used to boot stevia on an emulator or a real machine.

#### Requirements

- Bash shell
- Root privileges
- dd utility
- sfdisk utility (for Linux) or hdiutil & util-linux utility (for macOS, util-linux is needed for sfdisk because hdiutil only creates hybrid disks on newer versions afaik)
- newfs_msdos utility (for macOS)
- mkfs.vfat utility (for Linux)

#### Usage

Run the script as root (or just allow the make script to do its business):

```bash
sudo ./create_disk_image.sh
```

The script will create a disk image file named `disk.img` in the current directory. The disk image file will have the following characteristics:

- Size: 128 MiB
- Partition table: DOS
- Partition 1: FAT32, bootable, starts at sector 2048, contains the boot32 boot test file (`BOOT_386.bin`)
- Boot code: MBR (`mbr.bin`), VBR (`vbr.bin`), stage2 (sectors 1-63) (`stage2.bin`)

The script will also copy the necessary files from the `build` directory to the disk image file. If the files are not found, the script will exit with an error.

[^note]: **Note:** Please note that the assembly code is strictly targeting the Pentium 3 Katmai uArch, but it might run elsewhere. Your mileage may vary.
