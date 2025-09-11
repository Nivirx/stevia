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
[ORG 0x6C00]
[CPU KATMAI]
[WARNING -reloc-abs-byte]
[WARNING -reloc-abs-word]                   ; Yes, we use absolute addresses. surpress these warnings.
[map all build/mbr.map]
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

%undef __STEVIA_DEV_DEBUG

; BIOS will load 1st sector of boot drive to 0x7c00, init and relocate
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

    sub sp, 0x10                    ; local varible space (32 bytes)
    push bp                         ; setup top of stack frame

    xor cx, cx
    mov ch, 0x02                    ; 0x0200 in cx
    mov si, 0x7C00                  ; Current MBR Address (loaded here by BIOS)
    mov di, MBR_ENTRY               ; New MBR Address (our new relocation address)
    rep movsb                       ; copy 512 bytes from 0x0000:7c00 to 0x0000:MBR_ENTRY (6C00 as of writing)

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

        clc
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
        push ax                                            ; drive_num (2)
        push 0x01                                          ; count (2)

        mov dword eax, dword [bx + PartEntry_t.lba_start]
        push dword eax                                     ; lba (4)

        push word 0x7C00                                   ; offset = 0x7c00 (2)
        push 0x00                                          ; segment = 0 (2)

        ; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
        ;                         uint32_t lba,
        ;                         uint16_t count, uint16_t drive_num)
        call read_disk_raw
        add sp, 0xC
    .goto_vbr:
        cmp word [VBR_ENTRY + 0x1FE], 0xAA55
        je main.sig_ok
        ERROR MBR_ERROR_NO_VBR_SIG              ; no signature present
    .sig_ok:  
        push PartTable_t_size                   ; len       
        push DiskSig                            ; src -> start of partition table
        push partition_table                    ; dst -> addr in bss 
        call kmemcpy                            ; copy partition table to bss
        add sp, 0x6
        
        mov si, word [bp - 4]                   ; partition_offset address
        mov dl, byte [bp - 2]                   ; pass drive # from BIOS to VBR in dl
        mov bx, partition_table                 ; partition_table address
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
; !!! end of MBR !!!
section .bss follows=.text
begin_bss:

align 16, resb 1
partition_table resb PartTable_t_size

align 16, resb 1
lba_packet resb LBAPkt_t_size

align 512, resb 1
stack_bottom resb 512 - 16                 ; 512 byte stack early on
stack_top:
mbr_redzone resb 16
end_bss: