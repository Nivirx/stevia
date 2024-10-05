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
%define fat32_ebpb            0x30A0
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

%macro DEBUG_HCF 0
    DEBUG_LOOP:
        hlt
        jmp short DEBUG_LOOP
%endmacro