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


; ## Generic Low mem map (from osdev wiki) ##
; start 	        end 	    size 	        type 	                    description
; Low Memory (the first MiB)
; 0x00000000 	0x000003FF 	1 KiB 	        RAM - partially unusable  	Real Mode IVT (Interrupt Vector Table)
; 0x00000400 	0x000004FF 	256 bytes 	    RAM - partially unusable 	BDA (BIOS data area)
; 0x00000500 	0x00007BFF 	almost 30 KiB 	RAM - free for use 	        Conventional memory
; 0x00007C00  	0x00007DFF 	512 bytes 	    RAM - partially unusable  	OS BootSector
; 0x00007E00 	0x0007FFFF 	480.5 KiB 	    RAM - free for use 	        Conventional memory
; 0x00080000 	0x0009FFFF 	128 KiB 	    RAM - partially unusable 	EBDA (Extended BIOS Data Area)
; 0x000A0000 	0x000FFFFF 	384 KiB 	    various (unusable) 	        Video memory, ROM Area

; ## A rough overview of high mem ##
; Idealy this needs to be probed with the E820 functions
; # Start                   # End           # size                      # description
;
; 0x00100000 	            0x00EFFFFF 	    0x00E00000 (14 MiB) 	    RAM -- free for use (if it exists) 	Extended memory
; 0x00F00000 	            0x00FFFFFF 	    0x00100000 (1 MiB) 	        Possible memory mapped hardware 	ISA Memory Hole 15-16MB
; 0x01000000 	            ???????? 	    ??? (whatever exists) 	    RAM -- free for use 	            More Extended memory
; 0xC0000000 (sometimes) 	0xFFFFFFFF 	    0x40000000 (1 GiB) 	 	    Memory mapped PCI devices, PnP NVRAM?, IO APIC/s, local APIC/s, BIOS, ...
; 0x0000000100000000  	    ???	            ??? (whatever exists) 	    RAM -- free for use (PAE/64bit)/More Extended memory
; ???????????????? 	        ??? 	        ??? 	                    Potentially usable for memory mapped PCI devices in modern hardware (but typically not, due to backward compatibility) 

; 0x2700 -> 0x28FF
%define disk_buffer           0x2700
; 0x2900 -> 0x2AFF
%define fat_buffer            0x2900
; 0x2B00 -> 0x2CFF
%define dir_buffer            0x2B00

; copy of partition table, 72 bytes
%define partition_table       0x3000
%define partition_table_SIZE  72

; copy of FAT32 BPB, 33 bytes (+1 to the next value to align to uint16_t)
;0x3048
%define fat32_bpb             0x3050
%define fat32_bpb_SIZE        33

; copy of FAT32 EBPB, 54 bytes
;0x306A
%define fat32_ebpb            0x3070
%define fat32_ebpb_SIZE       54

; FAT32 FSInfo, 512 bytes
;0x30A2
%define fat32_fsinfo          0x30B0
%define fat32_fsinfo_SIZE     512

; some stored state for the fat32 driver
;0x32A2
%define fat32_state           0x32B0
%define fat32_state_SIZE      32

; next free space is 0x32D0
%define fat32_nc_data         0x32D0
%define fat32_nc_data_size    16

; lba_packet for raw_disk_read
%define lba_packet            0x4000

%define BIOSMemoryMap         0x4200
%define SteviaInfo            0x4400


; High memory addresses for loading kernel (for use with unreal mode and 32bit override)

; file load buffer at 16MB
%define HMEM_load_buffer      0x1000000


;PhysicalAddress = Segment * 16 + Offset
%define SEG_TO_LINEAR(s,o) ((s << 4) + o)

; Offset = physical / (Segment * 16)
%define LINEAR_TO_OFFSET(p,s) ((p / (s << 4)))

; Seg = (physical - offset) / 16
%define LINEAR_TO_SEGMENT(p,o) ((p - o) >> 4)

; create normalized linear addres from seg:off (16:4)
; Segement = linear >> 4 (top 16 bits)
; offset = linear & 0x0F (low 4 bits)

; 20 bytes, passed to loaded kernel
struc SteviaInfoStruct_t
    .MemoryMapPtr      resd 1
    .MemoryMapEntries  resd 1
    .BPBDataPtr        resd 1
    .EBPBDataPtr       resd 1
endstruc

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

; moves argument into eax then pushes lower 16bits then upper 16 bits of of argument to stack
; eax is clobbered
%macro PUSH_DWORD 1
    mov eax, %1
    push ax
    shr eax, 16
    push ax
%endmacro

; same as PUSH_DWORD except no arguments, pushes EAX directly
%macro PUSH_DWORD_EAX 0
    push ax
    shr eax, 16
    push ax
%endmacro

; pops upper 16bits then lower 16bits into eax
; eax is clobbered
%macro POP_DWORD_EAX 0
    xor eax, eax
    pop ax
    shl eax, 16
    pop ax
%endmacro

; moves data on stack referenced by bp to eax
; stack must be organized as follows (tl;dr push lower 16bits first ie. PUSH_DWORD macro)
; MOV_DWORD_EAX 2
; STACK TOP (0x0)
;   upper_uint16  [bp-4]
;   lower_uint16 [bp-2]
;   --- [bp] ---
;   ...
; STACK BOTTOM
;
; first argument == starting offset from bp for lower 16bits
%macro MOV_DWORD_EAX 1
    mov ax, [bp-(%1+2)]
    shl eax, 16
    mov ax, [bp-%1]
    ; eax contains dword from stack
%endmacro

%macro DEBUG_HCF 0
    DEBUG_LOOP:
        hlt
        jmp short DEBUG_LOOP
%endmacro


; uint8_t* kmemset(void* dest, uint8_t val, size_t len);
kmemset:
    push    di             ; function uses di, so save it.
 
    mov     cx, [bp - 2]   ; size_t len
    mov     al, [bp - 4]   ; uint8_t val
    mov     di, [bp - 6]   ; void * ptr

    cld
    rep     stosb 
.endf:
    mov     ax, [bp - 6]  ; return pointer to dest
    pop     di            ; restore di
    ret

; uint8_t* kmemset(uint8_t* dest, uint8_t* src, size_t len);
; not overlap safe
kmemcpy:
    push di
    push si                 ; di, si are callee save

    mov cx, [bp - 2]        ; length
    mov si, [bp - 4]        ; source
    mov di, [bp - 6]        ; dest
    
    cld                     ; ensure we are incrementing
    rep movsb

.endf:
    mov ax, [bp - 6]        ; return pointer to dest
    pop si
    pop di
    ret