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
[ORG 0x7C00]
[CPU KATMAI]
[WARNING -reloc-abs-word]
%define __STEVIA_VBR

__ENTRY:
    jmp short (init_thunk - $$)
    nop
    
phy_bpb_start:
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

%include "cdecl16.inc"
%include "entry.inc"
%include "config.inc"
%include "mem.inc"
%include "error_codes.inc"
%include "fat32/bpb_offset_bx.inc"

; ###############
; End Section
; ###############

ALIGN 4, db 0x90
init:
    cli                         ; We do not want to be interrupted

    xor ax, ax                  ; 0 AX
    mov ds, ax                  ; Set segment registers to 0

    mov ss, ax                  ; Set Stack Segment to 0
    mov sp, EARLY_STACK_START   ; Setup stack
    mov bp, sp                  ; base ptr = stack ptr
    sub sp, 0x20                ; local varible space

    mov bx, VBR_ENTRY           ; move Bx to the new start of the initial boot sector
    sti                         ; all done with inital setup and relocation, reenable interupts

    jmp 0:main                  ; fix up cs:ip just in case and jump to relocated code

; ###############
;
; Extra/Shared Functions
;
; ###############

%include "kmem_func.inc"
%include "util/error_func.inc"

; ###############
; End Section
; ###############

ALIGN 4, db 0x90
main:
    mov byte [bp - 2], dl            ; boot_drive
    mov [bp - 4], si                 ; part_offset

.check_FAT_size:                     ; we only support a very specific setup of FAT32
    cmp dword [bsSectorHuge], 0      ; SectorsHuge will not be set if FAT12/16
    ja main.load_stage2
    ERROR VBR_ERROR_WRONG_FAT_SIZE
.load_stage2:                                          ; read sectors 1-63 to stage2 entry point
    mov ax, (fat32_bpb_SIZE + fat32_ebpb_SIZE)         ; size in byte
    push ax
    mov ax, (phy_bpb_start - 0x3)                      ; start of bpb - 0x3 for the jump short main at the start
    push ax
    mov ax, fat32_bpb                                  ; defined in memory.inc, destination
    push ax
    call kmemcpy                                       ; copy bpb to memory
    add sp, 0x6

    mov bx, fat32_bpb                                  ; bx now points to aligned memory structure

    movzx ax, byte [bp - 2]
    push ax                                            ; drive_num

    mov ax, STAGE2_SECTOR_COUNT
    push ax                                            ; count

    mov dword eax, 0x1
    push dword eax                                     ; lba

    xor ax, ax
    push ax                                            ; offset = 0

    ; 07E0:0 = 0x00007e00
    mov ax, STAGE2_ENTRY
    shr ax, 4
    push ax                                            ; segment = 7E0

    ; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
    ;                         uint32_t lba,
    ;                         uint16_t count, uint16_t drive_num)
    call read_disk_raw
    add sp, 0xC

.check_sig:
    mov ax, 0x7E0
    mov fs, ax
    cmp dword [fs:0x7FFC], 0xDEADBEEF
    je main.sig_ok

    ERROR VBR_ERROR_NO_SIGNATURE          ; no signature present in stage2
    
.sig_ok:
    mov si, [bp - 4]
    mov dx, [bp - 2]
    jmp 0:0x7E00

; ###############
; Required BIOS function(s)
; ###############

%include 'BIOS/func/ext_read.inc'

%assign bytes_remaining (420 - ($ - $$))
%warning VBR has bytes_remaining bytes remaining for code (MAX: 420 bytes)

times (510 - ($ - $$)) nop     ; Fill the rest of sector with nop

BootSig:
    dw 0xAA55                    ; Add boot signature at the end of bootloader