# stevia

This is a hobby project that aims to create a simple and lightweight operating system kernel. It is written in Assembly and C, and uses the GNU toolchain and Bochs emulator. The project features (or is hoping to feature):

- A boot loader that loads the kernel from a floppy disk image
- A kernel that implements basic functions, such as printing messages, handling interrupts, and managing memory
- A shell that allows user input and execution of commands
- A simple text editor that can create and edit files
- A calculator that can perform arithmetic operations
- (big maybe) micropython or some other language, cross compiling, etc...

## Installation

To build and run this project, you need to have the following tools installed:

- GNU Binutils
- GNU GCC
- GNU Make
- NASM
- Bochs (for testing, project might run on other virtuization/emulation platforms, I target the Pentium 3 Era with this project)

To build the project, run `make` in the root directory. This will generate a floppy disk image named `stevia.img` that contains the boot loader and the kernel.

To run the project, run `bochs -f bochsrc.txt` in the root directory. This will launch the Bochs emulator and load the floppy disk image. You should see the boot loader message, followed by the kernel message, and then the shell prompt.

To exit the emulator, press `Ctrl+C` in the terminal where you launched Bochs.

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

[^note]: **Note:** Please note that the assembly code is strictly targeting the Pentium 3 Katmai uArch, but it might run elsewhere. Your mileage may vary.
