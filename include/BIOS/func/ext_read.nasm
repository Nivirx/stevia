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

%ifndef __INC_EXT_READ
;Offset	Size	    Description
; 0	    1	    size of packet (16 bytes)
; 1	    1	    always 0
; 2	    2	    number of sectors to transfer (max 127 on some BIOSes)
; 4	    4	    transfer buffer (0xFFFF:0xFFFF)
; 8	    4	    lower 32-bits of starting 48-bit LBA
; 12	4	    upper 32-bits of starting 48-bit LBA
; needs to be aligned to a uint32_t
struc LBAPkt_t
    .size        resb 1
    .res0        resb 1
    .xfer_size   resw 1
    .offset      resw 1
    .segment     resw 1
    .lower_lba   resd 1
    .upper_lba   resd 1
endstruc

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
; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
;                         uint32_t lba,
;                         uint16_t count, uint8_t drive_num)
ALIGN 4, db 0x90
read_disk_raw:
    __CDECL16_PROC_ENTRY
.func:                         
    push LBAPkt_t_size        ; len
    push 0x00                 ; val = 0
    push lba_packet           ; dest = lba_packet address
    call kmemset              ; kmemset(dst, val, len)
    add sp, 0x06

    mov bx, lba_packet
    mov byte [bx + LBAPkt_t.size], LBAPkt_t_size

    mov ax, [bp + 12]
    mov word [bx + LBAPkt_t.xfer_size], ax

    mov eax, dword [bp + 8]
    mov dword [bx + LBAPkt_t.lower_lba], eax

    ; upper_lba is zero from kmemset
    ; TODO: possiblly support >32bit LBA addresses in the future
    ; this will limit us to (4GiB * sector size) of readable lba's from the disk

    mov ax, [bp + 6]
    mov word [bx + LBAPkt_t.offset], ax

    mov ax, [bp + 4]
    mov word [bx + LBAPkt_t.segment], ax

    xor ax, ax
    xor dx, dx
    mov si, bx                  ; ds:si LBAPkt_t
    mov ah, 0x42                ; call #
    mov dl, byte [bp + 14]      ; drive #             

    clc                         ; clear carry
    int 0x13
    jnc .endf

    %ifdef __STEVIA_MBR
    ERROR MBR_ERROR_DISK_READ_ERR
    %elifdef __STEVIA_VBR
    ERROR VBR_ERROR_DISK_READ_ERR
    %else
    ERROR STAGE2_MBR_DISK_READ_ERROR
    %endif
.endf:
    __CDECL16_PROC_EXIT
    ret

%endif
%define __INC_EXT_READ