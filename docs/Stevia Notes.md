# **Bootloader Documentation**

## **1. Calling Conventions**

### __cdecl16near Calling Convention

- **Purpose**: For calling near (within the same segment) functions in 16-bit code.
- **Stack Management**: Caller cleans up the stack after the function call.
- **Parameter Passing**: Parameters are pushed onto the stack from right to left.
- **Return Address**: A near return address (16-bit) is pushed onto the stack.
- **Return Value**: Placed in the AX register.

Example:

```assembly
; Caller
push param2
push param1
call near_func
add sp, 4  ; Clean up the stack (2 parameters * 2 bytes each)

; Callee (near_func)
near_func:
  push bp
  mov bp, sp
  ; Function body
  mov sp, bp
  pop bp
  ret
```

### __cdecl16far Calling Convention

- **Purpose**: For calling far (across different segments) functions in 16-bit code.
- **Stack Management**: Caller cleans up the stack after the function call.
- **Parameter Passing**: Parameters are pushed onto the stack from right to left.
- **Return Address**: A far return address (32-bit, consisting of a segment and an offset) is pushed onto the stack.
- **Return Value**: Placed in the AX register.

Example:

```assembly
; Caller
push param2
push param1
call far_func
add sp, 4  ; Clean up the stack (2 parameters * 2 bytes each)

; Callee (far_func)
far_func:
  push bp
  mov bp, sp
  ; Function body
  mov sp, bp
  pop bp
  retf  ; Far return
```

### **Key Differences**

- **Return Address**: `__cdecl16near` uses a 16-bit return address; `__cdecl16far` uses a 32-bit return address (segment:offset).
- **Function Scope**: `__cdecl16near` is for functions within the same segment; `__cdecl16far` is for functions that may be in different segments.
- **Return Instruction**: `__cdecl16near` uses `ret`; `__cdecl16far` uses `retf` (far return).

### **Register Usage**

#### Caller-Saved (Volatile) Registers

- **AX**: Accumulator, often used for return values.
- **CX**: Counter register.
- **DX**: Data register, used for I/O operations.
- **SI/DI**: String operation indexes.

#### Callee-Saved (Non-Volatile) Registers

- **BP**: Base pointer, used for stack frame management.
- **SP**: Stack pointer.
- **BX**: Base register.

Example:

```assembly
; Caller
push param2
push param1
call near_func
add sp, 4  ; Clean up the stack (2 parameters * 2 bytes each)

; Callee (near_func)
near_func:
  push bp
  mov bp, sp
  ; Save callee-saved registers if used
  push bx
  push si
  push di
  ; Function body
  ; Use AX, CX, DX freely
  ; Restore callee-saved registers
  pop di
  pop si
  pop bx
  mov sp, bp
  pop bp
  ret
```

## **2. E820 Memory Map Usage**

### **Address Range Descriptor Structure**

| Offset | Name          | Description                           |
|--------|---------------|---------------------------------------|
| 0      | BaseAddrLow    | Low 32 bits of base address           |
| 4      | BaseAddrHigh   | High 32 bits of base address          |
| 8      | LengthLow      | Low 32 bits of length in bytes        |
| 12     | LengthHigh     | High 32 bits of length in bytes       |
| 16     | Type           | Address type of this range            |

### **E820 Function Call**

#### Input

- **EAX**: Function code `E820h`.
- **EBX**: Continuation value for physical memory retrieval (0 for the first call).
- **ES:DI**: Buffer pointer to an Address Range Descriptor structure.
- **ECX**: Buffer size (minimum size 20 bytes).
- **EDX**: Signature ('SMAP').

#### Output

- **CF**: Carry flag (indicates success/failure).
- **EAX**: Signature ('SMAP').
- **ECX**: Buffer size (number of bytes returned).
- **EBX**: Continuation value for subsequent E820 calls.

### **Address Type Values**

| Value | Pneumonic            | Description                                               |
|-------|----------------------|-----------------------------------------------------------|
| 1     | AddressRangeMemory    | Available RAM usable by the operating system.             |
| 2     | AddressRangeReserved  | Reserved by the system, unusable by the operating system. |

## **3. Example Calculations**

### **Partition Offset**

- Partition 1 offset = LBA 0x800 = 0x100000
- `bsSectorSize = 512`

### **First FAT Sector**

- `first_fat_sector = bsResSector = 32 => (32*512) = 0x4000`
- `first_fat_sector = 0x100000 + 0x4000 = 0x104000`

### **Total FAT Sectors**

- `total_fat_sectors = fat_sectors * number_of_FATs = 2001 * 2 = 4002`
- `total_fat_size = total_fat_sectors * bsSectorSize = 0x1F4400`

### **First Data Sector**

- `first_data_sector = FatStartSector + FatAreaSize = 0x104000 + 0x1F4400 = 0x2F8400`

### **FAT Table Look Up**

```c
if the cluster we got from the table entry was cluster 354
fat_sector = 354 / 128 = 2
fat_entry  = 354 mod 128 = 98

so we load the 3rd (indexed from 0) fat table sector and read the 98th entry

```

Example:

```c

fat_table_offset = (first_fat_sector + 2) * 512
fat_table = *(fat_table_offset)
disk_read(fat_table[98])
```

## **4. Global Descriptor Table (GDT)**

### **Segment Attributes**

- **Pr**: Present bit (must be 1 for valid selectors).
- **Privl**: Privilege level (0 = kernel, 3 = user).
- **S**: Descriptor type (set for code/data segments, cleared for system segments).
- **Ex**: Executable bit (set if segment contains code).
- **DC**: Direction/Conforming bit (for data or code segments).
- **RW**: Readable/Writable (depends on segment type).

### **Granularity (Gr)**

- **Gr**: Granularity bit (0 = byte granularity, 1 = 4 KiB blocks).
- **Sz**: Size bit (0 = 16-bit mode, 1 = 32-bit mode).

### **GDT Entry Construction**

Each GDT entry is 8 bytes:

- First DWORD: Limit (0:15), Base (0:15)
- Second DWORD: Base (16:31), Attributes (8:12)

## **5. Memory Layout Example**

### **Low Memory (First MiB)**

| Start       | End         | Size            | Type                  | Description                      |
|-------------|-------------|-----------------|-----------------------|----------------------------------|
| 0x00000000  | 0x000003FF  | 1 KiB           | RAM (partially unusable) | Real Mode IVT (Interrupt Vector Table) |
| 0x00000400  | 0x000004FF  | 256 bytes       | RAM (partially unusable) | BDA (BIOS data area)             |
| 0x00000500  | 0x00007BFF  | almost 30 KiB   | RAM - free for use     | Conventional memory              |
| 0x00007C00  | 0x00007DFF  | 512 bytes       | RAM (partially unusable) | OS BootSector                    |
| 0x00007E00  | 0x0007FFFF  | 480.5 KiB       | RAM - free for use     | Conventional memory              |
| 0x00080000  | 0x0009FFFF  | 128 KiB         | RAM (partially unusable) | EBDA (Extended BIOS Data Area)   |
| 0x000A0000  | 0x000FFFFF  | 384 KiB         | various (unusable)     | Video memory, ROM Area           |

### **Extended Memory (Above 1 MiB)**

| Start       | End         | Size            | Description            |
|-------------|-------------|-----------------|------------------------|
| 0x00100000  | 0x00EFFFFF  | 14 MiB          | RAM - free for use     |
| 0x00F00000  | 0x00FFFFFF  | 1 MiB           | Possible memory-mapped hardware (ISA) |
| 0x01000000  | ?           | ?               | More extended memory   |
| 0xC0000000  | 0xFFFFFFFF  | 1 GiB           | Memory mapped PCI devices, BIOS, etc. |
| 0x0000000100000000 | ?     | ?               | RAM - usable in PAE/64-bit mode |
