# Stevia Bootloader

Stevia is a lightweight, hobby bootloader written in Assembly and C, designed for educational purposes. It targets 686+ and aims to be simple, approachable, and understandable, focusing on minimalism and core functionality.

## Features

### Currently Implemented

- **Stage 1 Bootloader**: Loads from the Master Boot Record (MBR) and prepares the system for Stage 2.
- **Stage 2 Bootloader**: Loads the kernel into memory and passes control.
- **Basic Filesystem Support**: Initial filesystem recognition (details depend on implemented filesystem support).
- **Memory Map Parsing**: Detects available memory regions via BIOS interrupts.
- **GDT Initialization**: Sets up the Global Descriptor Table for protected/unreal mode.

### Planned Features

- **Interrupt Descriptor Table (IDT) Setup**: Setting up basic interrupt handling.
- **Basic Keyboard Input**: Initial support for capturing keystrokes.
- **Filesystem Expansion**: Support for additional filesystems.

## Installation

### Prerequisites

To build and run Stevia, you will need the following tools installed on your system:

- **Assembler**: NASM for assembling bootloader components, you could use a compatible assembler but YMMV.
- **GNU make**: For building the project.
- **mtools**: To access the FAT partition
- **fdisk**: To create disk images and setup partition tables
- **dosfstools**: FAT32 creation
- **an x86 Emulator**: Bochs is the primary dev target, but stevia should run on any x86 emulator/hardware with at least a Pentium 3 and 640KiB of memory.

### Building and Running

1. **Clone the repository**:

   ```sh
   git clone https://github.com/Nivirx/stevia.git
   cd stevia
   ```

2. **Build the bootloader and create a bootable disk image**:

   ```sh
   make
   ```

3. **Run using Bochs**:

   ```sh
   bochs -q -f bochsrc.txt
   ```

## Project Goals

Stevia is intended to be a learning tool for those interested in low-level programming, focusing on understanding the basics of how a computer starts up and manages early system resources.

## License

This project is licensed under the GPLv3 License. See the [LICENSE](LICENSE.md) or [COPYING](COPYING) file for more details.
