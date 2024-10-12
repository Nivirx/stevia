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

%ifndef __INC_PART_TABLE

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
    .lba_start  resd 1
    .lba_length resd 1
endstruc

struc PartTable_t
    .partition1 resb PartEntry_t_size
    .partition2 resb PartEntry_t_size
    .partition3 resb PartEntry_t_size
    .partition4 resb PartEntry_t_size
endstruc

%endif
%define __INC_PART_TABLE