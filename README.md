# Stevia Bootloader

Stevia is a lightweight, hobby bootloader written in Assembly and C, designed for educational purposes. It targets x86 (Pentium III era) and aims to be simple, approachable, and understandable, focusing on minimalism and core functionality.

## Features

### Currently Implemented

- **Stage 1 Bootloader**: Loads from the Master Boot Record (MBR) and prepares the system for Stage 2.
- **Stage 2 Bootloader**: Loads the kernel into memory and passes control.
- **Basic Filesystem Support**: Initial filesystem recognition (details depend on implemented filesystem support).
- **Memory Map Parsing**: Detects available memory regions via BIOS interrupts.
- **GDT Initialization**: Sets up the Global Descriptor Table for protected mode.

### In Progress

- **Interrupt Descriptor Table (IDT) Setup**: Setting up basic interrupt handling.
- **Basic Keyboard Input**: Initial support for capturing keystrokes.

### Planned Features

- **Task Scheduling**: Basic round-robin task switching.
- **Filesystem Expansion**: Support for additional filesystems.
- **More Robust Driver Support**: Additional hardware drivers, such as for storage devices.

## Installation

### Prerequisites

To build and run Stevia, you will need the following tools installed on your system:

- **Assembler**: NASM for assembling bootloader components.
- **C Compiler**: GCC (cross-compiler recommended).
- **Emulator**: Bochs or QEMU for testing.
- **GNU Make**: For building the project.
- **Utilities**:
  - **dd**: For creating raw disk images.
  - **losetup** (Linux only): For associating loop devices with disk images.
  - **sfdisk**: For creating DOS disk partitions.
  - **mkfs.fat**: For formatting partitions as FAT32.
  - **hdiutil** (macOS only): For attaching disk images.
  - **newfs\_msdos** (macOS only): For formatting FAT32 partitions.

### Building and Running

1. **Clone the repository**:
   ```sh
   git clone https://github.com/Nivirx/stevia.git
   cd stevia
   ```
2. **Build the bootloader and create a bootable disk image**:
   ```sh
   sudo make
   ```
3. **Run using Bochs**:
   ```sh
   bochs -f bochsrc.txt
   ```

## Project Goals

Stevia is intended to be a learning tool for those interested in low-level programming, focusing on understanding the basics of how a computer starts up and manages early system resources. Contributions and forks are welcome, especially from those interested in expanding the functionality.

## Contributing

We welcome contributions! Feel free to open issues for bugs or feature requests, and submit pull requests for any improvements. Please ensure that your code follows the existing style and includes appropriate comments.

## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for more details.
