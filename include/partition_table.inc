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
    .signature resb 4
    .reserved  resb 2
    .partition1 resb PartEntry_t_size
    .partition2 resb PartEntry_t_size
    .partition3 resb PartEntry_t_size
    .partition4 resb PartEntry_t_size
endstruc

%endif
%define __INC_PART_TABLE