; Copyright (C) 2025 Elaina Claus
; 
;     This program is free software: you can redistribute it and/or modify
;     it under the terms of the GNU General Public License as published by
;     the Free Software Foundation, either version 3 of the License, or
;     (at your option) any later version.
; 
;     This program is distributed in the hope that it will be useful,
;     but WITHOUT ANY WARRANTY; without even the implied warranty of
;     MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;     GNU General Public License for more details.
; 
;     You should have received a copy of the GNU General Public License
;     along with this program.  If not, see <https://www.gnu.org/licenses/>.

[BITS 16]
[ORG 0x0500]                            ; IF YOU CHANGE ORG CHANGE THE SIGN OFFSET AT THE END
[CPU KATMAI]
[map all build/stage2.map]
[WARNING -reloc-abs-byte]
[WARNING -reloc-abs-word]
[WARNING -reloc-abs-dword]              ; Yes, we use absolute addresses. surpress these warnings.
%define __STEVIA_STAGE2
%define __STAGE2_SEGMENT 0x0000

; ###############
; Headers/Includes/Definitions
; ###############

%include "util/bochs_magic.inc"
%include "cdecl16.inc"
%include "entry.inc"
%include "config.inc"
%include "early_mem.inc"
%include "error_codes.inc"

%macro print_string 1
    mov ax, %1
    push ax
    call PrintString
    add sp, 0x2
%endmacro

section .text
begin_text:
; dl = byte boot_drive
; ax = word part_offset (active partition offset)
; si = ptr PartTable_t partition_table
; di = ptr FAT32_bpb_t fat32_bpb
ALIGN 4, db 0x90
init:
    cli                               ; We do not want to be interrupted

    ; these 4 are stored in the .data section and are effectivly const types
    mov [vbr_part_table_ptr], si             ; pointer to partition_table
    mov [vbr_fat32_bpb_ptr], di              ; pointer to fat32_bpb
    mov [boot_drive], dl                     ; copy boot_drive to globals
    mov [partition_offset], ax               ; copy partition_offset to globals

    mov ax, __STAGE2_SEGMENT          ; set all our segments to the configured segment, except es
    mov ds, ax                        ; *
    mov fs, ax                        ; *
    mov gs, ax                        ; *
    mov ss, ax
    
    ; Zero BSS section
    mov cx, (end_bss - begin_bss)     ; count = bss length

    mov ax, begin_bss
    shr ax, 4
    mov es, ax                        ; es = begining of bss section, remember to restore ES later

    xor ax, ax                        ; val = 0
    mov di, ax                        ; dst = 0 (start of segment)
    cld
    rep stosb 

    mov ax, __STAGE2_SEGMENT
    mov es, ax  
    ; done zeroing bss section

    mov sp, stack_top
    mov bp, sp
    sub sp, 0x10
    push bp                           ; setup a somewhat normal stack frame, minus a ret ptr

    sti
    jmp word __STAGE2_SEGMENT:main

; ###############
; Core Functions
; ###############

%include "util/kmem_func.nasm"
%include "util/kmemcpy5_func.nasm"
%include "util/kmemset4_func.nasm"
%include "util/error_func.nasm"

; ###############
; Functions
; ###############
%include "util/arena_alloc.nasm"

; ###############
; FAT32 Driver
; ###############

%include 'fat32/FAT32_SYS.inc'

; ###############
; BIOS functions
; ###############

%include 'BIOS/BIOS_SYS.inc'

; structures

struc SteviaInfoStruct_t
    .MemoryMapPtr      resd 1
    .MemoryMapEntries  resd 1
endstruc

struc EarlyBootStruct_t
    .partition_table resb PartTable_t_size
    .fat32_bpb       resb FAT32_bpb_t_size
    .fat32_ebpb      resb FAT32_ebpb_t_size
endstruc

ALIGN 4, db 0x90
main:
    __BOCHS_MAGIC_DEBUG
.check_sig:
    mov eax, dword [STAGE2_SIG]
    cmp eax, 0xDEADBEEF
    je main.stage2_main
    ERROR STAGE2_SIGNATURE_MISSING
.stage2_main:
    mov ax, PartTable_t_size
    push ax                                                 ; len = PartTable_t_size
    mov ax, word [vbr_part_table_ptr]                       ; src = ptr to vbr partition_table
    push ax
    mov ax, partition_table                                 ; dst
    push ax
    call kmemcpy                                            ; copy partition table data to .data section in stage2
    add sp, 0x6

    mov ax, (FAT32_bpb_t_size + FAT32_ebpb_t_size)          ; len
    push ax
    mov ax, word [vbr_fat32_bpb_ptr]                        ; src            
    push ax
    mov ax, fat32_bpb                                       ; dst
    push ax
    call kmemcpy                                            ; copy bpb & ebpb to memory
    add sp, 0x6

    call SetTextMode
    call disable_cursor_bios
    print_string HelloPrompt_info
    
    ; setup the early heap
    mov ax, early_heap_state    
    push ax                             ; address in bss of state structure
    call arena_init
    add sp, 0x2

    ; enable A20 gate
    call EnableA20
    print_string A20_Enabled_OK_info

    ; get system memory map
    call GetMemoryMap
    print_string MemoryMap_OK_info

    ; enter unreal mode (16 bit code, 32 bit flat memory model)
    call EnterUnrealMode
    print_string UnrealMode_OK_info

    ; FAT Driver setup
    call InitFATDriver
    print_string InitFATSYS_OK_info

    ;
    ; Find first cluster of bootable file
    call SearchFATDIR
    push dword eax      ; save first cluster of bootable file

    print_string FileFound_OK_info
    
    pop dword eax
    push dword eax      ; print Cluster of boot file
    call PrintDWORD     ; void PrintDWORD(uint32_t dword)
    add sp, 0x4

    ; TODO: using first cluster information, start loading the kernel to memory
    ; TODO: going to need an elf parser,  some unreal mode file buffer functions to move the data
hcf:
    ERROR STEVIA_DEBUG_OK

; ##############################
;
; SYSTEM CONFIGURATION FUNCTIONS
;
; ##############################
ALIGN 4, db 0x90
EnterUnrealMode:
    __BOCHS_MAGIC_DEBUG
    __CDECL16_ENTRY
    cli                                   ; no interrupts
.func:    
    lgdt [((__STAGE2_SEGMENT << 4) + unreal_gdt_info)]                 ; load unreal gdt

    mov eax, cr0                          
    or eax, 1                             ; set pmode bit
    mov cr0, eax                          ; switch to pmode

    ; set cs to a pm code segment (0x8) w/ the following
    jmp 0x0008:EnterUnrealMode.set_segs
    ;db 0xEA                               ; jmp far imm16:imm16
    ;dw EnterUnrealMode.set_segs           ; error_far_ptr
    ;dw 0x0008                             ; error_far_seg
.set_segs:
    mov ax, 0x10                          ; select descriptor 2
    mov ds, ax                            ; 10h = 0001_0000b
    mov es, ax                            ; es to big data

    mov fs, ax
    mov gs, ax                            ; extra segments to big data as well
.pm_start:
    ; code here is running in protected mode w/ descriptor 0x8
    ; insert any PM test code needed here
.pm_end:
    mov eax, cr0
    and eax, ~1                           ; toggle bit 1 of cr0
    mov cr0, eax                          ; back to realmode
    jmp 0x0000:EnterUnrealMode.endp
.endp:
    sti                                   ; re-enable interupts

    ; set ds, es to the STAGE2_SEGMENT, for our model (generally) ds == es == cs
    ; fs, gs & ss are all still huge data model, and the macro(s) "__REFLAT_xxx" exists
    ; to easily access data outside of 64KiB boundries using ds/es addressing
    __TINY_DS_ES

    __CDECL16_EXIT
    ret
end_text:

section .data follows=.text
align 512
begin_data:
; #############
;
; Strings
;
; #############
%define CRLF 0Dh, 0Ah

%macro define_cstr 2
    align 16
    %1_cstr:
        db %2, 00h
%endmacro

%macro define_info 2
    align 16
    %1_info:
        db %2, CRLF, 00h
%endmacro

define_info HelloPrompt, "Hello from Stevia Stage2!"
define_info A20_Enabled_OK, "A20 Enabled OK"
define_info MemoryMap_OK, "Memory map OK"
define_info UnrealMode_OK, "Unreal mode OK"
define_info FileFound_OK, "Found SFN entry for bootable binary, first cluster -> "
define_info InitFATSYS_OK, "FAT32 Driver Init..."

define_info SearchFATDIR, "Searching FAT DIR for bootable file..."
define_info NextFATCluster, "Attempting to find next FAT cluster..."
define_info ReadFATCluster, "Attempting to load next FAT"
define_info MaybeFound_Boot, "Maybe found a file...checking..."

define_cstr BootTarget, "BOOT    BIN"

align 16, db 0
BootTarget:
    db 'BOOT    BIN'

;
; pre-bss init globals (generally const...but there are exceptions)
;

align 8, db 0
boot_drive:
    db 0x00

align 8, db 0
partition_offset:
    dw 0x0000

align 8, db 0
vbr_fat32_bpb_ptr:
    dw 0x0000

align 8, db 0
vbr_part_table_ptr:
    dw 0x0000

align 16, db 0
IntToHex_table:
    db '0123456789ABCDEF'

; see docs/gdt.txt for a quick refresher on GDT 
align 16, db 0
unreal_gdt_info:
    unreal_gdt_size: dw (unreal_gdt_end - unreal_gdt_start) - 1
    unreal_gdt_ptr:  dd ((__STAGE2_SEGMENT << 4) + unreal_gdt_start)
unreal_gdt_start:
    ; entry 0 (null descriptor)
    dq 0                    ; first entry is null

    ; entry 1 (0x8) (16bit code 64KiB limit)
    dd 0x0000FFFF        ; Base Address(15:0) 31:16, Segment Limit(15:0) 15:0
    db 0x00              ; Base Address 23:16
    db 1001_1010b        ; Access Byte: Present, ring0, S = 1, executable (1), non-conforming, readable, Accessed
    db 0000_0000b        ; Flags: GR = 4KiB, attr = <DB/L/Avl>, Granularity = 4KiB & 16:19 of limit
    db 0x00              ; Base Address 31:24

    ; entry 2 (0x10) (16bit data segment with 4 GiB flat mapping)
    dd 0x0000FFFF        ; Base Address(15:0) 31:16, Segment Limit(15:0) 15:0
    db 0x00              ; Base Address(23:16)
    db 1001_0010b        ; Access Byte: Present, ring0, S = 1, data (0), non-confirming, writable, present
    db 1000_1111b        ; Flags: GR = 4KiB, attr = <16-bit/?/?>, Granularity = 4KiB & 16:19 of limit
    db 0x00              ; Base Address(31:24)
unreal_gdt_end:

align 16, db 0
gdt32_info:
    gdt32_size: dw (gdt32_end - gdt32_start) - 1
    gdt32_ptr:  dd ((__STAGE2_SEGMENT << 4) + gdt32_start)
gdt32_start:
    dq 0
    .gdt32_code:
        dw 0xFFFF       ; code segment (RX)
        dw 0x0000
        db 0x00
        db 1001_1000b   ; Access: readable, executable
        db 1100_1111b   ; 4KB granularity, 32-bit
        db 0x00
    .gdt32_data:        ; data segment (RW)
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_0010b    ; Access: readable, writable
        db 1100_1111b    ; 4KB granularity, 32-bit
        db 0x00
    .gdt32_stack:        ; Stack segment (RW)
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_0010b    ; Access: readable, writable
        db 1100_1111b    ; 4KB granularity, 32-bit
        db 0x00
    .gdt32_ro_data:      ; Read-only data segment (RO)
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_0000b    ; Access: readable, not writable
        db 1100_1111b    ; 4KB granularity, 32-bit
        db 0x00
gdt32_end:

align 16,db 0
BUILD_NASM_VER:
    db "Stevia Stage2 built with NASM - ", __NASM_VER__, 00h

align 16,db 0
BUILD_DATETIME:
    db 'Assembled - ', __DATE__, ' ', __TIME__, 00h

align 16,db 0
BUILD_GIT_VER:
    db __GIT_VER__, 00h

align 16,db 0
BUILD_GIT_HASH:
    db __GIT_HASH__, 00h
end_data:

%assign bytes_remaining ((MAX_STAGE2_BYTES - 4) - (($ - $$) + (end_text - begin_text)))
%warning STAGE2 has bytes_remaining bytes remaining for code/data (MAX: MAX_STAGE2_BYTES)

; section start location needs to be a 'critical expression'
; i.e resolvable at build time, we are setting 0x7E00 as the offset since
section .sign start=((MAX_STAGE2_BYTES - 512) + 0x0500)
times ((512 - 4) - ($ -$$) ) db 0x90     ; nop
STAGE2_SIG: dd 0xDEADBEEF                ; Signature to mark the end of the stage2

section .bss follows=.sign
begin_bss:
; structures

align 16, resb 1
partition_table: 
    resb PartTable_t_size

align 16, resb 1
fat32_bpb: 
    resb FAT32_bpb_t_size
fat32_ebpb: 
    resb FAT32_ebpb_t_size

align 16, resb 1
fat32_nc_data: 
    resb 16

align 16, resb 1
lba_packet: 
    resb LBAPkt_t_size

align 16, resb 1
fat32_state:
    resb FAT32_State_t_size

align 16, resb 1
SteviaInfo:
    resd 4

align 16, resb 1
early_heap_state:
    resb ArenaStateStruc_t_size
;
; post-bss init globals
;

;
; large continuous allocations
;
align 16, resb 1
disk_buffer:
    resb 512
fat_buffer:
    resb 512
dir_buffer:
    resb 512
fat_fsinfo:
    resb 512

; TODO: this will hold 42 entries from the map function
; the e820 function needs to check that it doesn't overflow
; but realisticly 42 entries is enough for dev work
align 16, resb 1
%define BIOSMemoryMap_SIZE 1024
BIOSMemoryMap:
    resb BIOSMemoryMap_SIZE

align 16, resb 1
stack_bottom:
    resb 1024
stack_top:
end_bss: