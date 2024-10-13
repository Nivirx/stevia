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

%ifndef __INC_EARLY_MEM


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

;PhysicalAddress = Segment * 16 + Offset
%define SEG_TO_LINEAR(s,o) ((s << 4) + o)

; Offset = physical / (Segment * 16)
%define LINEAR_TO_OFFSET(p,s) ((p / (s << 4)))

; Seg = (physical - offset) / 16
%define LINEAR_TO_SEGMENT(p,o) ((p - o) >> 4)

%endif
%define __INC_EARLY_MEM