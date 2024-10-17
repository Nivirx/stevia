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
[WARNING -reloc-abs-byte]
[WARNING -reloc-abs-word]
[map all vbr.map]                   ; Yes, we use absolute addresses. surpress these warnings.
%define __STEVIA_VBR
section .text
__ENTRY:
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
%include "early_mem.inc"
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
    
    sub sp, 0x20                    ; local varible space
    push bp

    sti                             ; all done with inital setup and relocation, reenable interupts

    jmp 0:main                      ; fix up cs:ip just in case and jump to relocated code

; ###############
; Extra/Shared Functions
; ###############

%include "util/kmem_func.nasm"
%include "util/error_func.nasm"

; ###############
; End Section
; ###############

;
; byte boot_drive @ bp - 2
; word part_offset @ bp - 4
; ptr partition_table
ALIGN 4, db 0x90
main:
    mov byte [bp - 2], dl                 ; boot_drive
    mov word [bp - 4], si                 ; part_offset
    mov word [bp - 6], bx                 ; partition_table

.load_fs_data:
    mov ax, PartTable_t_size                           ; count=
    push ax                                             
    mov ax, [bp - 6]                                   ; src= ptr partition_table
    push ax
    mov ax, partition_table                            ; dst=      
    push ax
    call kmemcpy                                       ; copy partition table data
    add sp, 0x6

    mov ax, (FAT32_bpb_t_size + FAT32_ebpb_t_size)     ; size in byte, should be 90 bytes
    push ax
    mov ax, __ENTRY                                    
    push ax
    mov ax, fat32_bpb                                  ; 
    push ax
    call kmemcpy                                       ; copy bpb & ebpb to memory
    add sp, 0x6

.check_FAT_size:                     ; we only support a very specific setup of FAT32
    mov bx, fat32_bpb
    test word [bx + FAT32_bpb_t.unused2_ZERO_word], 0      ; TotSectors16 will not be set if FAT32
    jz main.load_stage2
    ERROR VBR_ERROR_WRONG_FAT_SIZE
.load_stage2:
    ; read sectors 1-(MAX_STAGE2_BYTES / 512) to stage2 entry point
    movzx ax, byte [bp - 2]
    push ax                                            ; drive_num

    mov ax, STAGE2_SECTOR_COUNT
    push ax                                            ; count

    mov dword eax, 0x1
    push dword eax                                     ; lba

    mov ax, STAGE2_ENTRY
    push ax                                            ; offset

    xor ax, ax
    push ax                                            ; segment = 0

    ; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
    ;                         uint32_t lba,
    ;                         uint16_t count, uint16_t drive_num)
    call read_disk_raw
    add sp, 0xC
.enter_stage2:
    mov dl, byte [bp - 2]               ; boot_drive
    mov si, word [bp - 4]               ; part_offset
    mov bx, partition_table
    mov dx, fat32_bpb
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

section .bss follows=.text
begin_bss:

align 16, resb 1
partition_table resb PartTable_t_size

align 16, resb 1
fat32_bpb resb FAT32_bpb_t_size
fat32_ebpb resb FAT32_ebpb_t_size

align 16, resb 1
fat32_nc_data resb 16

align 16, resb 1
lba_packet resb LBAPkt_t_size

align 512, resb 1
stack_bottom resb 512                  ; 512b stack early on
stack_top:
vbr_redzone resb 32
end_bss: