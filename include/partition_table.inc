;    Copyright (C) 2023  Elaina Claus
;
;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <https://www.gnu.org/licenses/>.


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