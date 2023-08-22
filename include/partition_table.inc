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

; Partition table entry format
; Off.  Size.       Description
;0x00 	  1 	Drive attributes (bit 7 set = active or bootable)
;0x01 	  3 	CHS Address of partition start
;0x04 	  1 	Partition type
;0x05 	  3 	CHS address of last partition sector
;0x08 	  4 	LBA of partition start
;0x0C 	  4 	Number of sectors in partition 

struc PartEntry_t
    .attributes resb 1
    .chs_start  resb 3
    .part_type  resb 1
    .chs_end    resb 3
    .lba_start  resb 4
    .lba_length resb 4
endstruc

struc PartTable_t
    .partition1 resb 16
    .partition2 resb 16
    .partition3 resb 16
    .partition4 resb 16
endstruc


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