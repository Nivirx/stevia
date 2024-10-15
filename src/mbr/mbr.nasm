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
[ORG 0x8C00]
[CPU KATMAI]
[WARNING -reloc-abs-byte]
[WARNING -reloc-abs-word]                   ; Yes, we use absolute addresses. surpress these warnings.
[map all mbr.map]
%define __STEVIA_MBR
jmp short (init - $$)
nop

; ###############
; Headers/Includes/Definitions
; ###############

%include "util/bochs_magic.inc"
%include "cdecl16.inc"
%include "entry.inc"
%include "config.inc"
%include "error_codes.inc"
%include "partition_table.inc"
%include "fat32/fat32_structures.inc"

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
    
    ; Zero BSS section
    mov cx, (end_bss - begin_bss)     ; count = bss length                    
    mov ax, begin_bss
    mov di, ax                        ; es:di is dest
    xor ax, ax
    cld
    rep stosb                         ; zero bss section

    sub sp, 0x20                    ; local varible space
    push bp                         ; setup top of stack frame

    xor cx, cx
    mov ch, 0x02                    ; 0x0200 in cx
    mov si, 0x7C00                  ; Current MBR Address (loaded here by BIOS)
    mov di, MBR_ENTRY               ; New MBR Address (our new relocation address)
    rep movsb                       ; copy 512 bytes from 0x0000:7c00 to 0x0000:MBR_ENTRY (7A00 as of writing)

    sti

    jmp 0:main

; ###############
; Extra/Shared Functions
; ###############

%include "util/kmem_func.nasm"
%include "util/error_func.nasm"

;
; bp - 2 : uint8_t boot_drive
; bp - 4 : uint16_t part_offset
ALIGN 4, db 0x90
main:
    mov byte [bp - 2], dl            ; BIOS passes drive number in DL

    .check_disk:
        cmp byte [bp - 2], 0x80
        jae main.check_extentions

        ERROR MBR_ERROR_DISK_T_ERR

    .check_extentions:
        xor ax, ax
        mov ah, 0x41
        mov bx, 0x55AA
        mov dl, 0x80
        int 0x13
        jnc main.find_active
        ERROR MBR_ERROR_NO_INT32E                         ; no extended function support

    .find_active:
        mov bx, PartEntry1                      ; base = first partition
        mov cx, 4                               ; there are only 4 entries
    .find_active_L0:
        mov al, byte [bx + PartEntry_t.attributes]
        test al, 0x80                           ; 0x80 == 1000_0000b
        jnz main.active_found
        add bx, 0x10                            ; add 16 bytes to offset
        loop main.find_active_L0

        ERROR MBR_ERROR_NO_NO_BOOT_PART

    .active_found:
        mov ax, bx
        sub ax, DiskSig                         ; leaves us with the offset relative from start of table
                                                ; this gives us an offset from the begining of the partition table
        mov word [bp - 4], ax                   ; update part_offset
    .read_data:
        movzx ax, byte [bp - 2]
        push ax                                            ; drive_num

        mov ax, 0x1
        push ax                                            ; count

        mov dword eax, dword [bx + PartEntry_t.lba_start]
        push dword eax                                     ; lba

        mov ax, VBR_ENTRY
        push ax                                            ; offset = 0x7c00

        xor ax, ax
        push ax                                            ; segment = 0

        ; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
        ;                         uint32_t lba,
        ;                         uint16_t count, uint16_t drive_num)
        call read_disk_raw
        add sp, 0xC
    .goto_vbr:
        cmp word [VBR_ENTRY + 0x1FE], 0xAA55
        je main.sig_ok
        ERROR MBR_ERROR_NO_VBR_SIG                        ; no signature present
    .sig_ok:
        mov ax, PartTable_t_size                
        push ax
        mov ax, DiskSig                         ; start of partition table
        push ax
        mov ax, partition_table                 
        push ax
        call kmemcpy                            ; copy partition table to bss
        add sp, 0x6
        
        mov si, word [bp - 4]
        mov dl, byte [bp - 2]
        mov bx, partition_table
        __BOCHS_MAGIC_DEBUG
        jmp word 0x0000:VBR_ENTRY

; ###############
;
; BIOS Functions
;
; ###############

%include 'BIOS/func/ext_read.nasm'

%assign bytes_remaining (440 - ($ - $$))
%warning MBR has bytes_remaining bytes remaining for code (MAX: 440 bytes)
times ((512 - 72) - ($ - $$)) nop     ; Fill the rest of sector with nop

DiskSig:
    times 4 db 0x00
Reserved:
    dw 0x0000
PartEntry1:
    times 16 db 0x00
PartEntry2:
    times 16 db 0x00
PartEntry3:
    times 16 db 0x00
PartEntry4:
    times 16 db 0x00
BootSig:
    dw 0xAA55                    ; Add boot signature at the end of bootloader

section .bss follows=.text
begin_bss:

align 16, resb 1
partition_table resb PartTable_t_size

align 16, resb 1
lba_packet resb LBAPkt_t_size

align 512, resb 1
stack_bottom resb 512                  ; 512 byte stack early on
stack_top:
mbr_redzone resb 32
end_bss: