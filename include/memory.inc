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