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
[CPU 686]
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
%include "error_codes.inc"

section .text
begin_text:
; dl = byte boot_drive
ALIGN 16, db 0x90
init:
    cli                               ; We do not want to be interrupted
    mov [u8_BootDrive], dl              ; copy boot_drive to globals

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

%include "partition_table.inc"
%include "fat32/fat32_structures.inc"
;%include 'fat32/FAT32_SYS.inc'

; ###############
; BIOS functions
; ###############

%include 'BIOS/BIOS_SYS.inc'

; structures

struc SteviaInfoStruct_t
    .p16_E820MemoryMapPtr       resw 1
    .u16_E820MMapEntryCount     resw 1
    .p16_MbrPtr                 resw 1
    .p16_VbrPtr                 resw 1
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
    call SetTextMode
    call disable_cursor_bios

    ; setup the early heap
    __CDECL16_CALL_ARGS early_heap_state
    __CDECL16_CALL ArenaInit, 1

    __CDECL16_CALL_ARGS pszHelloPrompt
    __CDECL16_CALL PrintString, 1
 
    ; setup heap space for mbr data
    __CDECL16_CALL_ARGS 0x200, 0x10
    __CDECL16_CALL ArenaAlloc, 2
    mov word [SteviaInfo + SteviaInfoStruct_t.p16_MbrPtr], ax

    push ax                                         ; dst
    movzx ax, byte [u8_BootDrive]                     
    push ax                                         ; boot_drive
    __CDECL16_CALL ReadMbrData, 2                   ; fill mbr buffer

    ; setup heap space for vbr data
    __CDECL16_CALL_ARGS 0x200, 0x10
    __CDECL16_CALL ArenaAlloc, 2
    mov word [SteviaInfo + SteviaInfoStruct_t.p16_VbrPtr], ax

    push ax                                         ; dst
    movzx ax, byte [u8_BootDrive]
    push ax                                         ; boot_drive
    __CDECL16_CALL ReadVbrData, 2                   ; fill vbr buffer

    ; enable A20 gate
    call EnableA20
    __CDECL16_CALL_ARGS pszA20EnabledOk
    __CDECL16_CALL PrintString, 1

    ; get system memory map
    call GetMemoryMap
    __CDECL16_CALL_ARGS pszMemoryMapOk
    __CDECL16_CALL PrintString, 1

    ; enter unreal mode (enter PM w/ 16 bit code, 32 bit flat memory model & return to real)
    ; ds, es will be set to the 64KiB STAGE2_SEGMENT, fs/gs will be flat/huge memory (4GiB)
    ; use __REFLAT macros to re-flat ds/es for easy transfers to >1MiB
    ; NOTE: if you modify a segment register you will need to re-unreal it
    call EnterUnrealMode
    __CDECL16_CALL_ARGS pszUnrealModeOk
    __CDECL16_CALL PrintString, 1

    ; FAT Driver setup
    ;call InitFATDriver
    ;__CDECL16_CALL_ARGS pszInitFAT32Ok
    ;__CDECL16_CALL PrintString, 1

    ;
    ; Find first cluster of bootable file
    ;call SearchFATDIR
    ;push dword eax      ; save first cluster of bootable file

    ;__CDECL16_CALL_ARGS pszFileFoundMsg
    ;__CDECL16_CALL PrintString, 1

    ;pop dword eax
    ;push dword eax      ; print Cluster of boot file
    ;call PrintDWORD     ; void PrintDWORD(uint32_t dword)
    ;add sp, 0x4

    ; TODO: using first cluster information, start loading the kernel to memory
    ; TODO: going to need an elf parser,  some unreal mode file buffer functions to move the data
hcf:
    ERROR STEVIA_DEBUG_OK

; ##############################
;
; SYSTEM CONFIGURATION FUNCTIONS & MACROS
;
; ##############################

; int read_mbr(int boot_drive, void* dst)
; destination buffer needs 512 bytes of space
ReadMbrData:
    __CDECL16_PROC_ENTRY
.proc:
    ; read mbr on boot drive to memory (for the partition table)
    movzx ax, byte [bp + 4]
    push ax                                 ; drive_num (2)
    push 0x01                               ; count (2)

    push dword 0x0                          ; lba (4)

    push word [bp + 6]                      ; offset = dst
    push __STAGE2_SEGMENT                   ; this segment 
    call read_disk_raw
    add sp, 0xC
.check_sig:
    mov bx, [bp + 6]
    cmp word [bx + 0x1FE], 0xAA55           ; check for bytes at end
    jne ReadMbrData.error_nosign
    ; TODO: this needs more error checking, zero checking, check the sig a bunch of stuff...
.endp:
    __CDECL16_PROC_EXIT
    ret
.error_nosign:
    ERROR STAGE2_ERROR_BAD_MBR

; int read_vbr(int boot_drive, void* buf)
; read vbr on boot partition to memory (for fat bpb/ebpb)
ReadVbrData:
    __CDECL16_PROC_ENTRY
.proc:
    mov bx, SteviaInfo
    mov ax, word [bx + SteviaInfoStruct_t.p16_MbrPtr]      
    add ax, DISK_PARTITION_TABLE_OFFSET                           
    mov bx, ax                              ; offset to part table

    mov cx, 4                               ; only checking 4 entries
    mov si, PartEntry_t.attributes
.find_active_L0:
    mov al, byte [bx + si]
    test al, 0x80                           ; 0x80 == 1000_0000b
    je ReadVbrData.active_found
    add bx, PartEntry_t_size                ; next part entry's attributes
    loop ReadVbrData.find_active_L0
    jmp ReadVbrData.error_noactive
.active_found:
    movzx ax, byte [bp + 4]
    push ax                                 ; drive_num (2)
    push 0x01                               ; count (2)

    mov eax, dword [bx + PartEntry_t.lba_start]
    push dword eax                          ; lba (4)

    push word [bp + 6]                      ; offset = dst
    push __STAGE2_SEGMENT                   ; this segment 
    call read_disk_raw
    add sp, 0xC
    ; vbr (with fat bpb/ebpb) is at the buffer now
.check_sig:
    mov bx, [bp + 6]
    cmp word [bx + 0x1FE], 0xAA55           ; check for bytes at end
    jne ReadVbrData.error_nosign
.check_FAT_size:
    test word [bx + FAT32_bpb_t.unused2_ZERO_word], 0      ; TotSectors16 will not be set if FAT32
    jnz ReadVbrData.error_badfatfs
.endp:
    __CDECL16_PROC_EXIT
    ret
.error_noactive:
    ERROR STAGE2_ERROR_BAD_VBR
.error_nosign:
    ERROR STAGE2_ERROR_BAD_VBR
.error_badfatfs:
    ERROR STAGE2_ERROR_BAD_VBR

; set ds and es segments back to the base of the loader
%ifnmacro __TINY_DS_ES
%macro __TINY_DS_ES 0
    mov ax, __STAGE2_SEGMENT
    mov ds, ax
    mov es, ax
%endmacro
%endif

; for copying between locations in high memory
%ifnmacro __REFLAT_DS_ES
%macro __REFLAT_DS_ES 0
    cli                                   ; no interrupts   
    lgdt [((__STAGE2_SEGMENT << 4) + UnrealGdtInfo)]                 ; load unreal gdt

    mov eax, cr0                          
    or eax, 1                             ; set pmode bit
    mov cr0, eax                          ; switch to pmode
    jmp short $+2                         ; i-cache flush

    mov ax, 0x10                          ; select descriptor 2
    mov ds, ax
    mov es, ax               

    mov eax, cr0
    and eax, ~1                           ; toggle bit 1 of cr0
    mov cr0, eax                          ; back to realmode
    jmp short $+2                         ; i-cache flush
    sti

%endmacro
%endif

; for copying from low memory to high memory (ds on a real segment, es in flat mode)
%ifnmacro __REFLAT_ES
%macro __REFLAT_ES 0
    cli                                   ; no interrupts   
    lgdt [((__STAGE2_SEGMENT << 4) + UnrealGdtInfo)]                 ; load unreal gdt

    mov eax, cr0                          
    or eax, 1                             ; set pmode bit
    mov cr0, eax                          ; switch to pmode
    jmp short $+2                         ; i-cache flush

    mov ax, 0x10                          ; select descriptor 2
    mov es, ax                            

    mov eax, cr0
    and eax, ~1                           ; toggle bit 1 of cr0
    mov cr0, eax                          ; back to realmode
    jmp short $+2                         ; i-cache flush
    sti

%endmacro
%endif

ALIGN 4, db 0x90
EnterUnrealMode:
    __CDECL16_PROC_ENTRY
    cli                                   ; no interrupts
.proc:    
    lgdt [((__STAGE2_SEGMENT << 4) + UnrealGdtInfo)]                 ; load unreal gdt

    mov eax, cr0                          
    or eax, 1                             ; set pmode bit
    mov cr0, eax                          ; switch to pmode

    ; set cs to a pm code segment (0x8) w/ the following
    jmp 0x0008:EnterUnrealMode.set_segs
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
    jmp __STAGE2_SEGMENT:EnterUnrealMode.endp
.endp:
    sti                                   ; re-enable interupts

    ; set ds, es to the STAGE2_SEGMENT, for our model (generally) ds == es == cs
    ; fs, gs & ss are all still huge data model, and the macro(s) "__REFLAT_xxx" exists
    ; to easily access data outside of 64KiB boundries using ds/es addressing
    __TINY_DS_ES

    __CDECL16_PROC_EXIT
    ret
end_text:

section .data follows=.text
align 16, db 0

begin_data:
; #############
;
; Strings
;
; #############
%define CRLF 0Dh, 0Ah

%macro define_cstr 2-*
%if %0 > 2
    align 16
    %1:
        db %{2:-1}, 00h
%else
    align 16
    %1:
        db %2, 00h
%endif
%endmacro

define_cstr pszHelloPrompt, "Hello from Stevia Stage2!", CRLF
define_cstr pszA20EnabledOk, "A20 Enabled OK", CRLF
define_cstr pszMemoryMapOk, "Memory map OK", CRLF
define_cstr pszUnrealModeOk, "Unreal mode OK", CRLF
define_cstr pszInitFAT32Ok, "FAT32 Driver Init OK", CRLF

define_cstr pszFileFoundMsg, "Found SFN entry for bootable binary, first cluster -> ", CRLF
define_cstr pszSearchFAT32DirMsg, "Searching FAT fs for bootable file...", CRLF
define_cstr pszReadFAT32ClusterMsg, "Attempting to load next FAT", CRLF
define_cstr pszMaybeFoundBootMsg, "Maybe found a file...checking...", CRLF

define_cstr BootTarget, "BOOT    BIN"

; set to boot_drive passed from BIOS almost first thing in init
align 4, db 0
u8_BootDrive:
    db 0x00

align 16, db 0
IntToHex_table:
    db '0123456789ABCDEF'

; see docs/gdt.txt for a quick refresher on GDT 
align 16, db 0
UnrealGdtInfo:
    u16_UnrealGdt_size: dw (UnrealGdtEnd - UnrealGdtStart) - 1
    pF_UnrealGdtPtr:  dd ((__STAGE2_SEGMENT << 4) + UnrealGdtStart)
UnrealGdtStart:
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
UnrealGdtEnd:

align 16, db 0
Gdt32Info:
    gdt32_size: dw (Gdt32End - Gdt32Start) - 1
    gdt32_ptr:  dd ((__STAGE2_SEGMENT << 4) + Gdt32Start)
Gdt32Start:
        dq 0            ; null segment
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
Gdt32End:

align 16,db 0
BuildNasmVer:
    db "Stevia Stage2 built with NASM - ", __NASM_VER__, 00h

align 16,db 0
BuildDateTime:
    db 'Assembled - ', __DATE__, ' ', __TIME__, 00h

align 16,db 0
BuildGitVer:
    db __GIT_VER__, 00h

align 16,db 0
BuildGitHash:
    db __GIT_HASH__, 00h
end_data:

%define SIG_BYTES 0x4

; Optional: fail fast if we overflowed
%if ((MAX_STAGE2_BYTES - 512) - (($ - $$) + (end_text - begin_text))) < 0
    %error "Stage2 overflow: code+data exceed MAX_STAGE2_BYTES - SIG_BYTES"
%endif

; section start location needs to be a 'critical expression'
; i.e resolvable at build time, we are setting 0x0500 as the offset since
section .sign start=((MAX_STAGE2_BYTES - 512) + 0x0500)
times ((512 - 4) - ($ -$$) ) db 0x90     ; nop
STAGE2_SIG: dd 0xDEADBEEF                ; Signature to mark the end of the stage2
; !!! END Stage2 .text/.data !!!
align 16, resb 1
section .bss follows=.sign
begin_bss:

; structures
align 16, resb 1
SteviaInfo:
    resd SteviaInfoStruct_t_size

align 16, resb 1
early_heap_state:
    resb ArenaStateStruc_t_size

align 16, resb 1
lba_packet: 
    resb LBAPkt_t_size

;
; large continuous allocations
;

; TODO: this will hold 42 entries from the map function
; the e820 function needs to check that it doesn't overflow
; but realisticly 42 entries is enough for dev work
align 16, resb 1
%define BIOSMemoryMap_SIZE 1024
BIOSMemoryMap:
    resb BIOSMemoryMap_SIZE

align 16, resb 1
stack_bottom:
    resb 2048
stack_top:
end_bss:

; Pad to the cap (emits nothing in the file; only increases .bss virtual size).
; If BSS_SIZE > BSS_MAX_BYTES, this becomes negative and NASM errors out.
%define BSS_MAX_BYTES 0x2000
resb (BSS_MAX_BYTES - (end_bss - begin_bss))

; Optional: keep a label for later math:
bss_cap_end: