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
[ORG 0x7C00]
[CPU KATMAI]
[WARNING -reloc-abs-byte]
[WARNING -reloc-abs-word]
[map all build/vbr.map]                   ; Yes, we use absolute addresses. surpress these warnings.
%define __STEVIA_VBR
section .text
__ENTRY:
phy_bpb_start:
    jmp short (init - $$)
    nop

; fill BPB area with 0x00 since we skip writing this part to disk
; but we need it for the 'jmp short entry; nop' above
times 33 db 0x00
phy_ebpb_start:
; fill eBPB area with 0x00 since we skip writing this part to disk
times 54 db 0x00

; ###############
;
; Headers/Includes/Definitions
;
; ###############

%include "util/bochs_magic.inc"
%include "cdecl16.inc"
%include "entry.inc"
%include "config.inc"
%include "error_codes.inc"
%include "partition_table.inc"
%include "fat32/fat32_structures.inc"

%undef __STEVIA_DEV_DEBUG

; dl = boot_drive
; si = part_offset
; bx = partition_table location from mbr
ALIGN 4
init:
    cli                             ; We do not want to be interrupted
    xor ax, ax                      
    mov ds, ax                      ; Set segment registers to 0x0000
    mov es, ax
    mov fs, ax
    mov gs, ax
    
    mov ss, ax                      ; Set Stack Segment to 0
    mov sp, end_bss                 ; Setup stack
    mov bp, sp                      ; base ptr = stack ptr

    ; zero bss section
    mov cx, (end_bss - begin_bss)     ; count = bss length                    
    mov ax, begin_bss
    mov di, ax                        ; es:di is dest
    xor ax, ax
    cld
    rep stosb                          
    
    sub sp, 0x10                    ; local varible space (32 bytes)
    push bp

    sti                             ; all done with inital setup and relocation, reenable interupts

    jmp 0:main                      ; fix up cs:ip just in case and jump to relocated code

; ###############
; Extra/Shared Functions
; ###############

%include "util/kmem_func.nasm"
%include "util/error_func.nasm"

;
; byte boot_drive @ bp - 2
; word part_offset @ bp - 4
; ptr partition_table
ALIGN 4, db 0x90
main:
    mov byte [bp - 2], dl                                  ; boot_drive
.check_FAT_size:                                           ; we only support a very specific setup of FAT32
    mov bx, phy_bpb_start
    test word [bx + FAT32_bpb_t.unused2_ZERO_word], 0      ; TotSectors16 will not be set if FAT32
    jz main.load_stage2
    ERROR VBR_ERROR_WRONG_FAT_SIZE
.load_stage2:
    ; read sectors 1-(MAX_STAGE2_BYTES / 512) to stage2 entry point
    movzx ax, byte [bp - 2]
    push ax                                            ; drive_num

    push STAGE2_SECTOR_COUNT                           ; count

    mov dword eax, 0x1
    push dword eax                                     ; lba

    push STAGE2_ENTRY                                  ; offset
    push 0x00                                          ; segment = 0

    ; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
    ;                         uint32_t lba,
    ;                         uint16_t count, uint16_t drive_num)
    call read_disk_raw
    add sp, 0xC
.enter_stage2:
    mov dl, byte [bp - 2]                ; byte boot_drive
    jmp word 0x0000:STAGE2_ENTRY

; ###############
; Required BIOS function(s)
; ###############

%include 'BIOS/func/ext_read.nasm'

%assign bytes_remaining (420 - ($ - $$))
%warning VBR has bytes_remaining bytes remaining for code (MAX: 420 bytes)

times (510 - ($ - $$)) nop     ; Fill the rest of sector with nop

BootSig:
    dw 0xAA55                    ; Add boot signature at the end of bootloader
; !!! END VBR !!!

section .bss follows=.text
begin_bss:

align 16, resb 1
lba_packet resb LBAPkt_t_size

align 512, resb 1
stack_bottom resb (1024 - 16)                  ; 512b stack early on
stack_top:
vbr_redzone resb 16
end_bss: