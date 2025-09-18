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

%ifndef __INC_FAT32_STRUCT

; ## FAT32 Info ##
;   total_sectors = bsSectorsHuge
;   fat_size = bsSectorsPerFat
; 
;       first_data_sector = bsResSectors + (bsFATs * bsSectPerFAT);
;       total_data_sectors = bsSectorsHuge - (bsResSectors + (bsFATs * bsSectPerFAT));
;       total_clusters = total_data_sectors / bsSectsPerClust
;       first_fat_sector = bsResSectors
;
; ## FAT32 Table information ##
;
;   fat_offset = active_cluster * 4
;   fat_sector = first_fat_sector + (fat_offset / sector_size)
;   entry_offset = fat_offset % sector_size
;   
;   table_value = fat_table[entry_offset] & 0x0FFF_FFFF
;
; ## FAT32 Directory Entries ##
;   root_dir_cluster = bsRootDirCluster
;   root_dir_sectors = ? (this is varible on FAT32 since the root dir is a cluster chain)
;   first_sector_of_cluster = ((cluster - 2) * bsSectPerClust) + first_data_sector;

; BPB Information
; Size   Description
;   8 	OEM identifier.
;   2 	The number of Bytes per sector (remember, all numbers are in the little-endian format).
;   1 	Number of sectors per cluster.
;   2 	Number of reserved sectors. The boot record sectors are included in this value.
;   1 	Number of File Allocation Tables (FAT's) on the storage media. Often this value is 2.
;   2 	Number of directory entries (must be set so that the root directory occupies entire sectors).
;   2 	The total sectors in the logical volume. 
;            If this value is 0, it means there are more than 65535 sectors in the volume, 
;            and the actual count is stored in the Large Sector Count entry at 0x20.
;
;   1 	This Byte indicates the media descriptor type.
;   2 	Number of sectors per FAT. FAT12/FAT16 only.
;   2 	Number of sectors per track.
;   2 	Number of heads or sides on the storage media.
;   4 	Number of hidden sectors. (i.e. the LBA of the beginning of the partition.)
;   4 	Large sector count. This field is set if there are more than 65535 sectors in the volume, 
;                           resulting in a value which does not fit in the Number of Sectors entry at 0x13. 

struc FAT32_bpb_t
    .u16_BytesPerSector      resw 1
    .u8_SectorsPerCluster    resb 1
    .u16_ReservedSectors     resw 1
    .u8_FatCount             resb 1
    .u16_RootEntryCount16    resw 1      ; Root dir entry count field, 0 on fat32
    .u16_TotalSectors16      resw 1      ; total number of sectors size, 0 on fat32
    .u8_MediaDesc            resb 1
    .u16_FATSize16           resw 1      ; FAT size 16, 0 on fat32
    .u16_SectorsPerTrack     resw 1
    .u16_HeadCount           resw 1
    .u32_HiddenSectors       resd 1
    .u32_TotalSectors32      resd 1
endstruc

; EBPB Information (FAT32)
; Size   Description
;   4 	Sectors per FAT. The size of the FAT in sectors.
;   2 	Flags.
;   2 	FAT version number. The high byte is the major version and the low byte is the minor version. FAT drivers should respect this field.
;   4 	The cluster number of the root directory. Often this field is set to 2.
;   2 	The sector number of the FSInfo structure.
;   2 	The sector number of the backup boot sector.
;   12 	Reserved. When the volume is formated these bytes should be zero.
;   1 	Drive number. The values here are identical to the values returned by the BIOS interrupt 0x13. 0x00 for a floppy disk and 0x80 for hard disks.
;   1 	Flags in Windows NT. Reserved otherwise.
;   1 	Signature (must be 0x28 or 0x29).
;   4 	Volume ID 'Serial' number. Used for tracking volumes between computers. You can ignore this if you want.
;   11 	Volume label string. This field is padded with spaces.
;   8 	System identifier string. Always "FAT32   ". The spec says never to trust the contents of this string for any use.

struc FAT32_ebpb_t
    .u32_FATSize32         resd 1      ; size of *each* fat in sectors, total = FATSz * (# of FATs)
    .u16_ExtFlags          resw 1
    .u16_FSVersion         resw 1
    .u32_RootDirCluster    resd 1
    .u16_FSInfoSector      resw 1
    .u16_BkBootSector      resw 1
    .reserved0             resb 12
    .u8_DriveNumber        resb 1
    .u8_NtFlags            resb 1
    .u8_Signature          resb 1
    .u32_VolumeId          resd 1
    .volume_label          resb 11
    .system_ident          resb 8      ; Should always be 'FAT32   '
endstruc

; ## Standard 8.3 structure ###
;   Offset      Length (Bytes)      Description
;   0               11              8.3 file name. The first 8 characters are the name and the last 3 are the extension.
;   11              1               File Atrributes
;                                      READ_ONLY=0x01 
;                                      HIDDEN=0x02 
;                                      SYSTEM=0x04 
;                                      VOLUME_ID=0x08 
;                                      DIRECTORY=0x10 
;                                      ARCHIVE=0x20 
;                                      LFN=READ_ONLY|HIDDEN|SYSTEM|VOLUME_ID == 0x0F
;
;   12              1                NT Reserved
;   13              1                Creation time in tenths of a second.
;   14              2                File creation time, Hour 5 bits, Minutes 6 bits, Seconds 5 Bits, multiply seconds by 2
;   16              2                File creation date, Year 7 bits, month 4 bits, day 5 bits
;   18              2                Last Accessed date. same format at creation date
;   20              2                High 16 bits of entry's first cluster
;   22              2                Last modification time. same format at creation time
;   24              2                Last modification date. same format as creation date
;   26              2                Low 16 bits of entry's first cluster
;   28              4                File size in bytes

; FSInfo 
; 0 	0x0 	4 	    Lead signature (must be 0x41615252 to indicate a valid FSInfo structure)
; 4 	0x4 	480 	Reserved, these bytes should never be used
; 484 	0x1E4 	4 	    Another signature (must be 0x61417272)
; 488 	0x1E8 	4 	    Contains the last known free cluster count on the volume. If the value is 0xFFFFFFFF, then the free count is unknown and must be computed. 
;                       However, this value might be incorrect and should at least be range checked (<= volume cluster count)
;
; 492 	0x1EC 	4 	    Indicates the cluster number at which the filesystem driver should start looking for available clusters. 
;                       If the value is 0xFFFFFFFF, then there is no hint and the driver should start searching at 2. 
;                       Typically this value is set to the last allocated cluster number. As the previous field, this value should be range checked.
;
; 496 	0x1F0 	12 	    Reserved
; 508 	0x1FC 	4 	    Trail signature (0xAA550000)

struc FAT32_FSInfo_t
    .head_sig_dword         resd 1          ; 0x41615252
    .reserved1              resb 480        ; fill zero
    .body_sig_dword         resd 1          ; 0x61417272
    .free_count_dword       resd 1
    .next_free_dword        resd 1
    .reserved2              resb 12         ; fill zero
    .tail_sig_dword         resd 1          ; 0xAA550000
endstruc

struc FAT32_SFN_t
    .label_83               resb 11
    .attributes_byte        resb 1
    .nt_res_byte            resb 1
    .csec_byte              resb 1
    .ctime_word             resb 2
    .cdate_word             resb 2
    .adate_word             resb 2
    .cluster_16_high        resb 2
    .mtime_word             resb 2
    .mdate_word             resb 2
    .cluster_16_low         resb 2
    .size_dword             resb 4
endstruc

; ## Long file name (LFN) structure format ##
;
; 0 	1 	The order of this entry in the sequence of long file name entries. This value helps you to know where in the file's name the characters from this entry should be placed.
; 1 	10 	The first 5, 2-byte characters of this entry.
; 11 	1 	Attribute. Always equals 0x0F. (the long file name attribute)
; 12 	1 	Long entry type. Zero for name entries.
; 13 	1 	Checksum generated of the short file name when the file was created. The short filename can change without changing the long filename in cases where the partition is mounted on a system which does not support long filenames.
; 14 	12 	The next 6, 2-byte characters of this entry.
; 26 	2 	Always zero.
; 28 	4 	The final 2, 2-byte characters of this entry. 
;
; LFN entries are always placed immediately before their respective 8.3 entry
; 
; LAST_LFN_ENTRY == 0x40
; Max of 20 in sequence == 0x14

struc FAT32_LFN_t
    .order                  resb 1
    .lfn_first5             resb 10
    .attributes_8           resb 1
    .zero1                  resb 1
    .checksum               resb 1
    .lfn_next6              resb 12
    .zero2                  resb 2
    .lfn_last2              resb 4
endstruc

; 32 bytes
struc FAT32_State_t
    .first_data_sector_32     resd 1
    .first_fat_sector_32      resd 1
    .fat_size_32              resd 1
    .curr_FAT_cluster_32      resd 1
    .curr_dir_cluster_32      resd 1
    .curr_drive_lba_32        resd 1
endstruc

; 16 bytes
struc FAT32_NextClusterData_t
    .fat_offset               resd 1
    .fat_sector               resd 1
    .entry_offset             resd 1
    .reserved_1               resd 1
endstruc

; FAT32 Attributes
%define FAT32_ATTR_RO           0x01
%define FAT32_ATTR_HIDDEN       0x02
%define FAT32_ATTR_SYSTEM       0x04
%define FAT32_ATTR_VOLID        0x08
%define FAT32_ATTR_DIR          0x10
%define FAT32_ATTR_ARC          0x20

; LFN == RO | HIDDEN | SYSTEM | VOLID == 0x0F
%define FAT32_ATTR_LFN          0x0F

%endif
%define __INC_FAT32_STRUCT