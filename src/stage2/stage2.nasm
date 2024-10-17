; Copyright (c) 2023 Elaina Claus
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in all
; copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
; SOFTWARE.

[BITS 16]
[ORG 0x0500]                            ; IF YOU CHANGE ORG CHANGE THE SIGN OFFSET AT THE END
[CPU KATMAI]
[map all stage2.map]
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
; si = word part_offset (active partition offset)
; bx = ptr PartTable_t partition_table
; dx = ptr FAT32_bpb_t fat32_bpb
ALIGN 4, db 0x90
init:
    __BOCHS_MAGIC_DEBUG
    cli                               ; We do not want to be interrupted

    mov ax, __STAGE2_SEGMENT          ; set all our segments to the configured segment, excep es
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
    sub sp, 0x20
    push bp                           ; setup a somewhat normal stack frame, minus a ret ptr

    sti
    jmp word __STAGE2_SEGMENT:main

; ###############
; Functions
; ###############

%include "util/kmem_func.nasm"
%include "util/kmemcpy5_func.nasm"
%include "util/kmemset4_func.nasm"
%include "util/error_func.nasm"

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

; bp - 2 : byte boot_drive
; bp - 4 : word part_offset
; bp - 6 : ptr PartTable_t partition_table
; bp - 8 : ptr FAT32_bpb_t fat32_bpb
ALIGN 4, db 0x90
main:
    lea ax, [bp - 2]
    mov [boot_drive_ptr], ax

    lea ax, [bp - 4]
    mov [partition_offset_ptr], ax      ; setup pointers to boot_drive and partition offset on stack

    mov byte [bp - 2], dl               ; boot_drive (probably 0x80)
    mov word [bp - 4], si               ; partition_offset
    mov word [bp - 6], bx               ; partition_table_vbr
    mov word [bp - 8], dx               ; fat32_bpb_vbr
.check_sig:
    mov eax, dword [STAGE2_SIG]
    cmp eax, 0xDEADBEEF
    je main.stage2_main
    ERROR STAGE2_SIGNATURE_MISSING
.stage2_main:
    mov ax, PartTable_t_size
    push ax
    mov ax, [bp - 6]                                    ; ptr partition_table
    push ax
    mov ax, partition_table                                  
    push ax
    call kmemcpy                                       ; copy partition table data
    add sp, 0x6

    mov ax, (FAT32_bpb_t_size + FAT32_ebpb_t_size)         ; size in byte
    push ax
    mov ax, [bp - 8]                                    
    push ax
    mov ax, fat32_bpb                                  ; defined in memory.inc, destination
    push ax
    call kmemcpy                                       ; copy bpb & ebpb to memory
    add sp, 0x6

    call SetTextMode
    call disable_cursor
    print_string HelloPrompt_info
    
    ; enable A20 gate
    call EnableA20
    print_string A20_Enabled_OK_info

    ; enter unreal mode
    call EnterUnrealMode
    print_string UnrealMode_OK_info

    ; get system memory map
    call GetMemoryMap
    print_string MemoryMap_OK_info

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

; Prints a C-Style string (null terminated) using BIOS vga teletype call
; void PrintString(char* buf)
ALIGN 4, db 0x90
PrintString:
    __CDECL16_ENTRY
    mov di, [bp + 4]    ; first arg is char* 
.str_len:
    xor cx, cx         ; ECX = 0
    not cx             ; ECX = -1 == 0xFFFF
    xor ax, ax         ; search for al = 0x0

    cld
    repne scasb        ; deincrement cx while searching for al

    not cx             ; the inverse of a neg number = abs(x) - 1
    dec cx             ; CX contains the length of the string - nul byte at end
.print:
    mov si, [bp + 4]            ; source string
.print_L0:
    movzx ax, byte [si]
    push ax
    call PrintCharacter
    add sp, 0x2

    inc si
    dec cx

    jcxz PrintString.endp
    jmp PrintString.print_L0    ; Fetch next character from string
.endp:
    __CDECL16_EXIT
    ret                         ; Return from procedure

; Prints a single character
; void PrintCharacter(char c);
ALIGN 4, db 0x90
PrintCharacter:
    __CDECL16_ENTRY
.func:
    mov ax, [bp + 4] ; c
    mov dx, 0x00ff
    and ax, dx

    mov ah, 0x0E                ; INT 0x10, AH=0x0E call
    mov bx, 0x0007              ; BH = page no. BL =Text attribute 0x07 is lightgrey font on black background
    int 0x10                    ; call video interrupt
.endp:
    __CDECL16_EXIT
    ret

; TODO: fix the prolog, epilog and stack usage to confirm with cdecl16
; prints the hex representation of of val
; void PrintDWORD(uint32_t val);
ALIGN 4, db 0x90
PrintDWORD:
    __CDECL16_ENTRY
.func: 
    mov si, IntToHex_table
    mov ebx, 16     ; base-16

    mov eax, dword [bp + 4]     ;val

    xor edx, edx
    xor cx, cx
.next_digit:
    div ebx            ; dividend in edx:eax -> quotient in eax, remainder in edx
    push dx            ; save remainder
    inc cx

    xor dx, dx
    test eax, eax
    jnz PrintDWORD.next_digit

.zero_pad:
    cmp cx, 0x0008
    je PrintDWORD.print_stack
    xor ax, ax
    push ax
    inc cx
    jmp PrintDWORD.zero_pad

.print_stack:
    pop bx
    dec cx
    push cx

    movzx ax, byte [bx+si+0]    ; bx = index into Hex lookup table
    push ax
    call PrintCharacter
    add sp, 0x2

    pop cx

    jcxz PrintDWORD.endp
    jmp PrintDWORD.print_stack

.endp:
    __CDECL16_EXIT
    ret

; ##############################
;
; SYSTEM CONFIGURATION FUNCTIONS
;
; ##############################
ALIGN 4, db 0x90
EnterUnrealMode:
    __CDECL16_ENTRY
.func:
    cli                         ; no interrupts
    push ds                     ; save real mode data/stack selectors
    push es
    push fs
    push gs
    push ss

    push cs                               ; save real mode code selector
    xor ax, ax                            ;
    pop ax                                ; save cs to ax to setup far jump
    mov word [__UNREAL_SEGMENT], ax    

    lgdt [((__STAGE2_SEGMENT << 4) + unreal_gdt_info)]                 ; load unreal gdt

    mov eax, cr0                          
    or al,1                               ; set pmode bit
    mov cr0, eax                          ; switch to pmode
    jmp short $+2

    ;jmp far 0x0008:EnterUnrealMode.load_cs
    db 0xEA                               ; jmp far imm16:imm16
    dw EnterUnrealMode.load_cs            ; error_far_ptr
    dw 0x0008                             ; error_far_seg
.load_cs:
    mov bx, 0x10                          ; select descriptor 2
    mov ds, bx                            ; 10h = 0001_0000b
                  
    mov ss, bx
    mov es, bx
    mov fs, bx
    mov gs, bx                            ; other data/stack to index 2 (off 0x10)
    
    and al,0xFE                           ; toggle bit 1 of cr0
    mov cr0, eax                          ; back to realmode
    jmp short $+2
    
    ;jmp far 0x0008:EnterUnrealMode.unload_cs
    db 0xEA                               ; jmp far imm16:imm16
    dw EnterUnrealMode.unload_cs          ; error_far_ptr
__UNREAL_SEGMENT:
    dw 0x0000                             ; error_far_seg
EnterUnrealMode.unload_cs:
    pop ss
    pop gs
    pop fs
    pop es
    pop ds                                ; get back old segments
    sti
.endp:
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
    ALIGN 16
    %1_cstr:
        db %2, 00h
%endmacro

%macro define_info 2
    ALIGN 16
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

ALIGN 16, db 0
BootTarget_str:
    db 'BOOT    BIN'

ALIGN 16
IntToHex_table:
    db '0123456789ABCDEF'

; see docs/gdt.txt for a quick refresher on GDT 
ALIGN 16, db 0
unreal_gdt_info:
    unreal_gdt_size: dw (unreal_gdt_end - unreal_gdt_start) - 1
    unreal_gdt_ptr:  dd ((__STAGE2_SEGMENT << 4) + unreal_gdt_start)
unreal_gdt_start:
    ; entry 0 (null descriptor)
    dq 0                    ; first entry is null

    ; entry 1 (16bit code 64KiB limit)
    dd 0x0000FFFF        ; Base Address(15:0) 31:16, Segment Limit(15:0) 15:0
    db 0x00              ; Base Address 23:16
    db 1001_1010b        ; Access Byte: Present, ring0, S = 1, executable (1), non-conforming, readable, Accessed
    db 0000_0000b        ; Flags: GR = 4KiB, attr = <DB/L/Avl>, Granularity = 4KiB & 16:19 of limit
    db 0x00              ; Base Address 31:24

    ; entry 2 (16bit data segment with 4 GiB flat mapping)
    dd 0x0000FFFF        ; Base Address(15:0) 31:16, Segment Limit(15:0) 15:0
    db 0x00              ; Base Address(23:16)
    db 1001_0010b        ; Access Byte: Present, ring0, S = 1, data (0), non-confirming, writable, present
    db 1000_1111b        ; Flags: GR = 4KiB, attr = <16-bit/?/?>, Granularity = 4KiB & 16:19 of limit
    db 0x00              ; Base Address(31:24)
unreal_gdt_end:

ALIGN 16, db 0
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

ALIGN 8,db 0x00
BUILD_NASM_VER:
    db "Stevia Stage2 built with NASM - ", __NASM_VER__, 00h

ALIGN 8,db 0x00
BUILD_DATETIME:
    db 'Assembled - ', __DATE__, ' ', __TIME__, 00h

ALIGN 8,db 0x00
BUILD_GIT_VER:
    db __GIT_VER__, 00h

ALIGN 8,db 0x00
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
partition_table resb PartTable_t_size

align 16, resb 1
fat32_bpb resb FAT32_bpb_t_size
fat32_ebpb resb FAT32_ebpb_t_size

align 16, resb 1
fat32_nc_data resb 16

align 16, resb 1
lba_packet resb LBAPkt_t_size

align 16, resb 1
fat32_state:
    resb FAT32_State_t_size

align 16, resb 1
SteviaInfo:
    resd 4

;
; locals
;
boot_drive_ptr:
    resw 1
partition_offset_ptr:
    resw 1

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

align 16, resb 1
%define BIOSMemoryMap_SIZE 2048
BIOSMemoryMap:
    resb 2048

align 512, resb 1
stack_bottom:
    resb 1024
stack_top:
end_bss: