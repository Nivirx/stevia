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
[ORG 0x7A00]
[CPU KATMAI]
jmp short init
nop

%include "entry.inc"
init:
    cli                             ; We do not want to be interrupted

    xor ax, ax                      ; 0 AX
    mov ds, ax                      ; Set segment registers to 0
    mov es, ax                      ; *
    
    mov ss, ax                      ; Set Stack Segment to 0
    mov sp, STACK_START             
    sti

    jmp 0:main
    nop

%include "config.inc"
%include "memory.inc"
%include "partition_table.inc"
%include "errors.inc"

main:
    mov [boot_drive], dl            ; BIOS passes drive number in DL

    .check_disk:
        cmp dl, 0x80
        jae main.find_active
        ERROR MBR_ERROR_DISK_T_ERR

    .check_extentions:
        xor ax, ax
        mov bx, 0x55AA
        mov dl, byte [boot_drive]
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
        mov word [part_offset], ax              ; update part_offset
    .read_data:
        push bp
        mov bp, sp

        ;uint8_t read_disk_raw(size_t count, uint16_t buf_segment, uint16_t buf_offset, 
        ;                      uint16_t lower_lower_lba, uint16_t upper_lower_lba)
        mov eax, dword [bx + PartEntry_t.lba_start]
        ror eax, 16
        push ax

        ror eax, 16
        push ax         

        mov ax, VBR_ENTRY
        push ax

        xor ax, ax
        push ax        ; segment = 0  

        mov ax, 1
        push ax

        call read_disk_raw
        leave

        jnc main.goto_vbr
        ERROR MBR_ERROR_DISK_READ_ERR                     ; error in LBA read
    .goto_vbr:
        cmp word [VBR_ENTRY + 0x1FE], 0xAA55
        je main.sig_ok
        ERROR MBR_ERROR_NO_VBR_SIG                        ; no signature present
    
    .sig_ok:
        push bp
        mov bp, sp

        mov ax, partition_table_SIZE            ; 72 bytes of data
        push ax
        mov ax, DiskSig                         ; start of partition table
        push ax
        mov ax, partition_table                 ; defined in memory.inc
        push ax
        call kmemcpy                            ; copy partition table to memory
        leave

        xor ah, ah                              ; Set Video mode BIOS function
        mov al, 0x02                            ; 16 color 80x25 Text mode
        int 0x10                                ; Call video interrupt

        mov ah, 0x05                            ; Select active display page BIOS function
        xor al, al                              ; page 0
        int 0x10                                ; call video interrupt

        mov si, word [part_offset]
        mov dl, byte [boot_drive]
        jmp 0:0x7C00

; Wrapper for AH=0x42 INT13h (Extended Read)
;
; BIOS call details
; AH = 42h
; DL = drive number
; DS:SI -> disk address packet
;
; Return:
; CF clear if successful
; AH = 00h
; CF set on error
; AH = error code
; disk address packet's block count field set to number of blocks
; successfully transferred
;
;
; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, uint16_t lower_lower_lba, uint16_t upper_lower_lba)
; bp-0 = call frame
; bp-2 = upper_lower_lba
; bp-4 = lower_lower_lba
; bp-6 = offset
; bp-8 = segment
; bp-10 = count
; bp-12 = ret ptr
read_disk_raw:
    push si

    ; uint8_t* kmemset(void* dest, uint8_t val, size_t len);                      
    push bp
    mov bp, sp

    mov ax, 0x10
    push ax             ; len = 16 bytes
    xor ax, ax
    push ax             ; val = 0
    mov ax, lba_packet
    push ax             ; dest = lba_packet address

    call kmemset
    leave

    mov byte [lba_packet + LBAPkt_t.size], 0x10
    mov word [lba_packet + LBAPkt_t.xfer_size], STAGE2_SECTOR_COUNT

    mov ax, [bp-2]
    shl eax, 16
    mov ax, [bp-4]
    mov dword [lba_packet + LBAPkt_t.lower_lba], eax

    mov ax, [bp-6]
    mov word [lba_packet + LBAPkt_t.offset], ax

    mov ax, [bp-8]
    mov word [lba_packet + LBAPkt_t.segment], ax

    mov si, lba_packet
    mov ah, 0x42
    mov dl, byte [boot_drive]
    int 0x13
    jnc read_disk_raw.endf

    mov al, 'B'
    jmp error
.endf:
    pop si
    ret

; #############
;
; Locals
;
; #############

boot_drive:
    db 0x00
mbr_reserved1:
    db 0x00

; OFFSET from BEGINING of partition table, ie. DiskSig (-6 from PartEntry1)
part_offset:
    dw 0x0000

%assign bytes_remaining (440 - ($ - $$))
%warning MBR has bytes_remaining bytes remaining for code (MAX: 440 bytes)
times ((512 - 72) - ($ - $$)) nop     ; Fill the rest of sector with nop

DiskSig:
    times 4 db 0x00
Reserved1:
    db 0x00
Reserved2:
    db 0x00

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