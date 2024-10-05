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
;	0	    BaseAddrLow		Low 32 Bits of Base Address
;	4	    BaseAddrHigh	High 32 Bits of Base Address
;	8	    LengthLow		Low 32 Bits of Length in Bytes
;	12	    LengthHigh		High 32 Bits of Length in Bytes
;	16	    Type		Address type of  this range.
; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		Low 32 Bits of Base Address
;	4	    BaseAddrHigh	High 32 Bits of Base Address
;	8	    LengthLow		Low 32 Bits of Length in Bytes
;	12	    LengthHigh		High 32 Bits of Length in Bytes
;	16	    Type		Address type of  this range.
; Input:
;
;	EAX	Function Code	E820h
;	EBX	Continuation	Contains the "continuation value" to get the
;				next run of physical memory.  This is the
;				value returned by a previous call to this
;				routine.  If this is the first call, EBX
;				must contain zero.
;	ES:DI	Buffer Pointer	Pointer to an  Address Range Descriptor
;				structure which the BIOS is to fill in.
;	ECX	Buffer Size	The length in bytes of the structure passed
;				to the BIOS.  The BIOS will fill in at most
;				ECX bytes of the structure or however much
;				of the structure the BIOS implements.  The
;				minimum size which must be supported by both
;				the BIOS and the caller is 20 bytes.  Future
;				implementations may extend this structure.
;	EDX	Signature	'SMAP' -  Used by the BIOS to verify the
;				caller is requesting the system map
;				information to be returned in ES:DI.
;
; Output:
;
;	CF	Carry Flag	Non-Carry - indicates no error
;	EAX	Signature	'SMAP' - Signature to verify correct BIOS
;				revision.
;	ES:DI	Buffer Pointer	Returned Address Range Descriptor pointer.
;				Same value as on input.
;	ECX	Buffer Size	Number of bytes returned by the BIOS in the
;				address range descriptor.  The minimum size
;				structure returned by the BIOS is 20 bytes.
;	EBX	Continuation	Contains the continuation value to get the
;				next address descriptor.  The actual
;				significance of the continuation value is up
;				to the discretion of the BIOS.  The caller
;				must pass the continuation value unchanged
;				as input to the next iteration of the E820
;				call in order to get the next Address Range
;				Descriptor.  A return value of zero means that
;				this is the last descriptor
;
; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		Low 32 Bits of Base Address
;	4	    BaseAddrHigh	High 32 Bits of Base Address
;	8	    LengthLow		Low 32 Bits of Length in Bytes
;	12	    LengthHigh		High 32 Bits of Length in Bytes
;	16	    Type		Address type of  this range.
;
; The BaseAddrLow and BaseAddrHigh together are the 64 bit BaseAddress of this range.
; The BaseAddress is the physical address of the start of the range being specified.
;
; The LengthLow and LengthHigh together are the 64 bit Length of this range.
; The Length is the physical contiguous length in bytes of a range being specified.
;
; The Type field describes the usage of the described address range as defined in the table below.

; Value	       Pneumonic		           Description
;   1	    AddressRangeMemory	    This run is available RAM usable by the operating system.
;   2	    AddressRangeReserved	This run of addresses is in use or reserved by the system, and must not be used by the operating system.
; Other	    Undefined		        Undefined - Reserved for future use.

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

    mov dx, (BIOSMemoryMap >> 4)
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