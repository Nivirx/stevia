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

%ifndef __INC_E820MEMORY_MAP

; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		u32 - Low 32 Bits of Base Address
;	4	    BaseAddrHigh	u32 - High 32 Bits of Base Address
;	8	    LengthLow		u32 - Low 32 Bits of Length in Bytes
;	12	    LengthHigh		u32 - High 32 Bits of Length in Bytes
;	16	    Type		    u32 - Address type of  this range.
;   20      ExtType         u32 -  ACPI 3.0 extended type
struc AddressRangeDescStruct_t
    .BaseAddrLow    resd 1
    .BaseAddrHigh   resd 1
    .LengthLow      resd 1
    .LengthHigh     resd 1
    .Type           resd 1
    .ExtType        resd 1
endstruc

ALIGN 4, db 0x90
GetMemoryMap:
    __CDECL16_ENTRY
    push es         ; save segment register
.func:
    mov dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries], 0

    mov eax, 0xE820                         ; select 0xE820 function
    xor ebx, ebx                            ; Continuation value, 0 for the first call

    lea dx, [BIOSMemoryMap]
    shr dx, 4
    mov es, dx
    xor di, di                              ; (BIOSMemoryMap >> 4):0 makes di an index into BIOSMemoryMap

    mov ecx, AddressRangeDescStruct_t_size
    mov edx, 0x534D4150                     ; 'SMAP' magic
.loop_L1:
    int 0x15
    jc GetMemoryMap.error
    cmp eax, 0x534D4150
    jne GetMemoryMap.no_smap_returned
.no_error:
    inc dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries]

    cmp ecx, 20                             ; TODO: maybe this could be handled better than just panicing
    jb GetMemoryMap.nonstandard_e820        ; non-standard entry found 

    cmp ebx, 0
    je GetMemoryMap.endp                    ; 0 in ebx means we have reached the end of memory ranges

    add di, AddressRangeDescStruct_t_size   ; increment di to next descriptor
    mov edx, eax                            ; 'SMAP' to edx
    mov eax, 0xE820                         ; select E820 function
    jmp GetMemoryMap.loop_L1
.error:
    ERROR STAGE2_MM_E820_MISC_ERR
.nonstandard_e820:
    ERROR STAGE2_MM_E820_NONSTANDARD
.no_smap_returned:
    ERROR STAGE2_MM_E820_NO_SMAP
.endp:
    pop es
    __CDECL16_EXIT
    ret
    
%endif
%define __INC_E820MEMORY_MAP