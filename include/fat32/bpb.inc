;    Copyright (C) 2020  Bradley Claus
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


; BPB Information
; Off.  Hex Off. Size   Description
; 0 	 0x00 	   3 	The first three bytes EB 3C 90 disassemble to JMP SHORT 3C NOP. (The 3C value may be different.) 
; 3 	 0x03 	   8 	OEM identifier.
; 11 	 0x0B 	   2 	The number of Bytes per sector (remember, all numbers are in the little-endian format).
; 13 	 0x0D 	   1 	Number of sectors per cluster.
; 14 	 0x0E 	   2 	Number of reserved sectors. The boot record sectors are included in this value.
; 16 	 0x10 	   1 	Number of File Allocation Tables (FAT's) on the storage media. Often this value is 2.
; 17 	 0x11 	   2 	Number of directory entries (must be set so that the root directory occupies entire sectors).
; 19 	 0x13 	   2 	The total sectors in the logical volume. 
;                           If this value is 0, it means there are more than 65535 sectors in the volume, 
;                           and the actual count is stored in the Large Sector Count entry at 0x20.
;
; 21 	 0x15 	   1 	This Byte indicates the media descriptor type.
; 22 	 0x16 	   2 	Number of sectors per FAT. FAT12/FAT16 only.
; 24 	 0x18 	   2 	Number of sectors per track.
; 26 	 0x1A 	   2 	Number of heads or sides on the storage media.
; 28 	 0x1C 	   4 	Number of hidden sectors. (i.e. the LBA of the beginning of the partition.)
; 32 	 0x20 	   4 	Large sector count. This field is set if there are more than 65535 sectors in the volume, 
;                           resulting in a value which does not fit in the Number of Sectors entry at 0x13. 

;-- BPB (BIOS Parameter Block)
%define bsOemName           bx+0x03
%define bsBytesPerSect      bx+0x0B
%define bsSecsPerClust      bx+0x0D
%define bsResSectors        bx+0x0E
%define bsFATs              bx+0x10
%define bsRootDirEnts       bx+0x11
%define bsSectors           bx+0x13
%define bsMedia             bx+0x15
%define bsSectPerFAT1216    bx+0x16
%define bsSectPerTrack      bx+0x18
%define bsHeads             bx+0x1A
%define bsHidden            bx+0x1C
%define bsSectorHuge        bx+0x20
;-- End BPB

; EBPB Information (FAT32)
; Off.     Hex Off.     Size   Description
; 36 	    0x024 	      4 	Sectors per FAT. The size of the FAT in sectors.
; 40 	    0x028 	      2 	Flags.
; 42 	    0x02A 	      2 	FAT version number. The high byte is the major version and the low byte is the minor version. FAT drivers should respect this field.
; 44 	    0x02C 	      4 	The cluster number of the root directory. Often this field is set to 2.
; 48 	    0x030 	      2 	The sector number of the FSInfo structure.
; 50 	    0x032 	      2 	The sector number of the backup boot sector.
; 52 	    0x034 	      12 	Reserved. When the volume is formated these bytes should be zero.
; 64 	    0x040 	      1 	Drive number. The values here are identical to the values returned by the BIOS interrupt 0x13. 0x00 for a floppy disk and 0x80 for hard disks.
; 65 	    0x041 	      1 	Flags in Windows NT. Reserved otherwise.
; 66 	    0x042 	      1 	Signature (must be 0x28 or 0x29).
; 67 	    0x043 	      4 	Volume ID 'Serial' number. Used for tracking volumes between computers. You can ignore this if you want.
; 71 	    0x047 	      11 	Volume label string. This field is padded with spaces.
; 82 	    0x052 	      8 	System identifier string. Always "FAT32   ". The spec says never to trust the contents of this string for any use.
; 90 	    0x05A 	      420 	Boot code.
; 510 	    0x1FE 	      2 	Bootable partition signature 0xAA55. 

;-- EBPB (Extended BIOS Parameter Block)
%define bsSectorsPerFAT     bx+0x24
%define bsFlags             bx+0x28
%define bsFATVersion        bx+0x2A
%define bsRootDirCluster    bx+0x2C
%define bsFSInfoCluster     bx+0x30
%define bsBackupBootSector  bx+0x32
%define bsReserved1         bx+0x34
%define bsDriveNumber       bx+0x40
%define bsNTFlags           bx+0x41
%define bsFATSignature      bx+0x42
%define bsVolumeSerial      bx+0x43
%define bsVolumeLabel       bx+0x47
%define bsSystemIdent       bx+0x52
;-- End EBPB