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