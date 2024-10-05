; Copyright (c) 2024 Elaina Claus
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
;
; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, uint32_t lba)
; TODO: this needs validation
read_disk_raw:
    __CDECL16_ENTRY
.func:  
    mov ax, 0x10
    push ax             ; len = 16 bytes
    xor ax, ax
    push ax             ; val = 0
    mov ax, lba_packet
    push ax             ; dest = lba_packet address

    call kmemset        ; uint8_t* kmemset(void* dest, uint8_t val, size_t len);
    add sp, 0x6

    mov byte [lba_packet + LBAPkt_t.size], 0x10
    mov word [lba_packet + LBAPkt_t.xfer_size], 0x0001

    mov dword eax, [bp + 8]
    mov dword [lba_packet + LBAPkt_t.lower_lba], eax

    mov ax, [bp + 6]
    mov word [lba_packet + LBAPkt_t.offset], ax

    mov ax, [bp + 4]
    mov word [lba_packet + LBAPkt_t.segment], ax

    mov si, lba_packet
    mov ah, 0x42
    mov dl, byte [fat32_ebpb + FAT32_ebpb_t.drive_number_8]
    int 0x13
    jnc read_disk_raw.endp
    ERROR STAGE2_MBR_DISK_READ_ERROR
.endp:
    __CDECL16_EXIT
    ret