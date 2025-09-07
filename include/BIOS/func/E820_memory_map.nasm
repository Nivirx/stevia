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
    __CDECL16_PROC_ENTRY
    push es         ; save segment register
.func:
    mov dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries], 0

    
    xor ebx, ebx                            ; Continuation value, 0 for the first call

    mov dx, BIOSMemoryMap
    shr dx, 4
    mov es, dx
    xor di, di                              ; (BIOSMemoryMap >> 4):0 makes di an index into BIOSMemoryMap

    mov ecx, AddressRangeDescStruct_t_size  ; hard request ACPI 3.0 table versions (24 bytes)
    
.loop_L1:
    mov eax, 0x0000E820                     ; select 0xE820 function
    mov edx, 0x534D4150                     ; 'SMAP' magic
    clc                                     ; clear carry
    int 0x15
    jc GetMemoryMap.error
    cmp eax, 0x534D4150
    jne GetMemoryMap.no_smap_returned
.no_error:
    mov eax, dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries]
    add eax, 1
    mov dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries], eax

    cmp ecx, 20                             ; TODO: maybe this could be handled better than just panicing
    jb GetMemoryMap.nonstandard_e820        ; non-standard entry found 

    cmp ebx, 0
    je GetMemoryMap.endp                    ; 0 in ebx means we have reached the end of memory ranges

    ; setup for next loop, ebx is our next index and has already been set.
    xor ecx, ecx
    mov ecx, AddressRangeDescStruct_t_size  ; set ecx (requested_size) to 24 bytes
    add di, cx                              ; increment di to next descriptor so es:di points to the next free space

    jmp GetMemoryMap.loop_L1
.error:
    ERROR STAGE2_MM_E820_MISC_ERR
.nonstandard_e820:
    ERROR STAGE2_MM_E820_NONSTANDARD
.no_smap_returned:
    ERROR STAGE2_MM_E820_NO_SMAP
.endp:
    pop es
    __CDECL16_PROC_EXIT
    ret

PrintMemoryMap:
    __CDECL16_PROC_ENTRY
.func:
    mov eax, dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries]
    cmp eax, 0
    je PrintMemoryMap.endp          ; if entries == 0, exit

    mov ecx, eax                    ; store # of entries in counter
    mov eax, BIOSMemoryMap          ; address to start of e820 mmap

    push dword ecx
    push dword eax
    call FormatMemoryMapEntry
    add sp, 0x8

    ; ax contains segment offset to string after call
    ; TODO: going to need an allocator for strings and stuff...
    ; print_string strformat_buffer
    
.endp:
    __CDECL16_PROC_EXIT
    ret

FormatMemoryMapEntry:
    __CDECL16_PROC_ENTRY
.func:
    ; create a string buffer somewhere and return address to result string in ax
.endp:
    __CDECL16_PROC_EXIT
    ret


%endif
%define __INC_E820MEMORY_MAP