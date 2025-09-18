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

%ifndef __INC_FAT32_SYS

%include "partition_table.inc"
%include "fat32/bpb_offset_bx.inc"
%include "fat32/fat32_structures.inc"

; int read_mbr(int boot_drive, void* dst)
; destination buffer needs 512 bytes of space
FAT32_load_mbr:
    __CDECL16_PROC_ENTRY
.proc:
    ; read mbr on boot drive to memory (for the partition table)
    movzx ax, byte [bp + 4]
    push ax                                 ; drive_num (2)
    push 0x01                               ; count (2)

    push dword 0x0                          ; lba (4)

    push word [bp + 6]                      ; offset = dst
    push __STAGE2_SEGMENT                   ; this segment 
    call BIOS_int13h_ext_read
    add sp, 0xC
.check_sig:
    mov bx, [bp + 6]
    cmp word [bx + 0x1FE], 0xAA55           ; check for bytes at end
    jne ReadMbrData.error_nosign
    ; TODO: this needs more error checking, zero checking, check the sig a bunch of stuff...
.endp:
    __CDECL16_PROC_EXIT
    ret
.error_nosign:
    ERROR STAGE2_ERROR_BAD_MBR

; int read_vbr(int boot_drive, void* buf)
; read vbr on boot partition to memory (for fat bpb/ebpb)
; TODO: seperate validation and loading the sector, just check the 0xAA55 at the end here...
FAT32_load_vbr:
    __CDECL16_PROC_ENTRY
.proc:
    mov bx, SteviaInfo
    mov ax, word [bx + SteviaInfoStruct_t.p16_MbrPtr]      
    add ax, DISK_PARTITION_TABLE_OFFSET                           
    mov bx, ax                              ; offset to part table
    mov cx, 4                               ; only checking 4 entries

.find_active_L0:
    mov al, [bx + PartEntry_t.attributes]
    cmp al, 0x80                            ; 0x80 == 1000_0000b
    je ReadVbrData.check_fstype
    add bx, PartEntry_t_size                ; next part entry's attributes
    loop ReadVbrData.find_active_L0
    jmp ReadVbrData.error_noactive
.check_fstype:
    ; check for part_type = 0x0C (DOS 7.1+/W95 FAT32 w/ LBA) or 0x1C (Hidden 0x0C)
    __BOCHS_MAGIC_DEBUG
    mov al, [bx + PartEntry_t.part_type]
    cmp al, 0x0C
    je ReadVbrData.active_ok
    ; *or*
    cmp al, 0x1C
    je ReadVbrData.active_ok

    jmp ReadVbrData.error_badparttype        ; error if part_type != 0x1C or 0x0C
.active_ok:
    movzx ax, byte [bp + 4]
    push ax                                 ; drive_num (2)
    push 0x01                               ; count (2)

    mov eax, dword [bx + PartEntry_t.lba_start]
    push dword eax                          ; lba (4)

    push word [bp + 6]                      ; offset = dst
    push __STAGE2_SEGMENT                   ; this segment 
    call BIOS_int13h_ext_read
    add sp, 0xC
    ; vbr (with fat bpb/ebpb) is at the buffer now
.check_sig:
    mov bx, [bp + 6]
    cmp word [bx + 0x1FE], 0xAA55           ; check for bytes at end
    jne ReadVbrData.error_nosign
.check_FAT_sanity:
    ; we only quickly validate that this is *probably* a FAT32 volume

    add bx, 11                                             ; point bx at start of bpb (skip jmp code and ident)
    cmp word [bx + FAT32_bpb_t.u16_TotalSectors16], 0      ; TotalSectors16 should be 0 (use TotalSectors32 in bpb)
    jne ReadVbrData.error_totsectors

    cmp word [bx + FAT32_bpb_t.u16_FATSize16], 0           ; FatSize16 will be 0 if FAT32 (use FATSize32 in ebpb)
    jne ReadVbrData.error_fatsz

    cmp word [bx + FAT32_bpb_t.u16_RootEntryCount16], 0    ; root dir info is in data clusters on fat32
    jne ReadVbrData.error_rootdir
.endp:
    __CDECL16_PROC_EXIT
    ret
.error_noactive:
    ERROR STAGE2_VBR_E_ACTIVE
.error_nosign:
    ERROR STAGE2_VBR_E_SIGN
.error_totsectors:
    ERROR STAGE2_VBR_E_TOT
.error_fatsz:
    ERROR STAGE2_VBR_E_FATSZ
.error_rootdir:
    ERROR STAGE2_VBR_E_DIRENT
.error_badparttype:
    ERROR STAGE2_VBR_E_PARTTYPE

; what a 'mount' should probably do at this point...
; - read VBR at partition_lba
; - validate FAT32 signatures
; - fill fat32_bpb_t and compute derived fields
; - read FSInfo (free count/next free)
; - ???
; int fat32_mount(uint32_t partition_lba, fat32_bpb_t* out);
FAT32_mountfs:
    __CDECL16_PROC_ENTRY
.proc:
    ; mount: parse BPB, derive fat0_lba, data_lba, cluster0_lba.
.endp:
    __CDECL16_PROC_EXIT
    ret
.error:
    ERROR STEVIA_DEBUG_ERR


; int fat32_read_fat(const fat32_bpb_t* v, uint32_t clus, uint32_t* out);
FAT32_read_fat:
    __CDECL16_PROC_ENTRY
.proc:
    ; Read 32-bit FAT entry for cluster n
.endp:
    __CDECL16_PROC_EXIT
    ret
.error:
    ERROR STEVIA_DEBUG_ERR

; FAT32 entry is 32-bits; only low 28 bits are meaningful.
;   EOC if (val & 0x0FFFFFFF) >= 0x0FFFFFF8.
;   bad if == 0x0FFFFFF7.
;   free if == 0x00000000.
; int fat32_next_clus(const fat32_bpb_t* v, uint32_t clus, uint32_t* out_next);
FAT32_next_cluster:
    __CDECL16_PROC_ENTRY
.proc:
    ; Walk to next cluster in chain (validates EOC/bad)
.endp:
    __CDECL16_PROC_EXIT
    ret
.error:
    ERROR STEVIA_DEBUG_ERR

; e.g:
; uint64_t clus_to_lba(const fat32_bpb_t* v, uint32_t clus) {
;  return v->data_lba + (uint64_t)(clus - 2) * v->secs_per_clus;
; }
FAT32_clus_to_lba:
    __CDECL16_PROC_ENTRY
.proc:
    ; Convert cluster -> LBA of first sector of that cluster
.endp:
    __CDECL16_PROC_EXIT
    ret
.error:
    ERROR STEVIA_DEBUG_ERR

; Prototyping for now...

; TODO:
;   - bio_read_sectors (BIOS int13h LBA or CHS) + tiny sector cache.
;   - mount: parse BPB, derive fat0_lba, data_lba, cluster0_lba.
;   - FAT read: fat32_read_fat, fat32_next_clus.
;   - dir iterator (SFN only) + path lookup.
;   - file reader (fopen/fread) with cluster crossing.
;   - contiguity probe for speed (optional).
;   - (later) LFN support & code page handling.


; keep count <= 127
; int bio_read_sectors(uint64_t lba, uint16_t count, void* dst);
; will also need a bio_bios_int13h() (refactor ext_disk_read?)
; to abstract the backend 'sector getter'

; Error codes

; FS_OK 0
; FS_E_IO 1 (bio read failed)
; FS_E_FMT 2 (not FAT32 or bad BPB)
; FS_E_RANGE 3 (bad cluster or out of range)
; FS_E_END 4 (iterator end)
; FS_E_NOSUCH 5 (file/dir not found)
; FS_E_ISDIR 6 (opened dir as file)
; FS_E_TOOLONG 7 (path segment too long for 8.3)
; FS_E_UNSUPPORTED 8 (LFN present but disabled)
; FS_E_UNIMPLEMENTED 0xffff (procedure call unimplmented)

;
; Cache methods
;
; TODO: make a cache for ~4-8 disk sectors
; TODO: make a cache for FATs to avoid having to re-read the FAT
;
; // uses bio_read_sectors + cache
; int cache_read_sector(uint64_t lba, void* dst);   
; int cache_invalidate_all(void);

;
; SFN directories
;

; typedef struct {
;   char     name83[12];   // "FILENAMEEXT" (no dot), NUL-terminated
;   uint8_t  attr;         // ATTR_DIRECTORY=0x10, ATTR_VOLUME=0x08
;   uint32_t first_clus;   // (hi<<16)|lo
;   uint32_t size;         // bytes
; } fat32_dirent_sfn_t;

; typedef struct { // iterator
;   fat32_bpb_t* v;
;   uint32_t     cur_clus;
;   uint32_t     offs_in_clus; // byte offset
; } fat32_dir_iter_t;

;int fat32_dir_iter_open_root(fat32_bpb_t* v, fat32_dir_iter_t* it);
;int fat32_dir_iter_open(fat32_bpb_t* v, uint32_t first_clus, fat32_dir_iter_t* it);
;int fat32_dir_iter_next(fat32_dir_iter_t* it, fat32_dirent_sfn_t* out); // FS_OK or FS_E_END

; typedef struct {
;   uint32_t first_clus;
;   uint32_t size;
;   uint8_t  attr; // dir or file
; } fat32_node_t;

; split by / or \.
; start at root (v->root_clus).
; for each component:
; iterate directory with dir_iter_* to find case-insensitive 8.3 match (fold to upper and space-pad during compare).
; if final component: return file’s cluster & size.
; if intermediate: ensure ATTR_DIRECTORY and descend.
;
; int fat32_path_lookup(fat32_bpb_t* v, const char* path, fat32_node_t* out);

; an example 'do the thing' proc to load a file to a location
;
; // loads file at path into dst, up to max_bytes; returns size read.
; int fat32_load_file(fat32_bpb_t* v, const char* path, void* dst,
;                     uint32_t max_bytes, uint32_t* out_size);

; streamed reads
;
; typedef struct {
;   fat32_bpb_t* v;
;   uint32_t     first_clus;
;   uint32_t     cur_clus;
;   uint32_t     cur_clus_index;   // which cluster within file we’re on
;   uint32_t     size;             // total bytes
;   uint32_t     pos;              // current byte position
; } fat32_file_t;
; 
; 
; int fat32_fopen(fat32_bpb_t* v, const char* path, fat32_file_t* f);
; int fat32_fread(fat32_file_t* f, void* dst, uint32_t nbytes, uint32_t* out_read);
; int fat32_fseek(fat32_file_t* f, uint32_t new_pos);   // forward seeks only is fine
; int fat32_fclose(fat32_file_t* f);


; an example 'do the thing' proc to load a file to a location
;
; // loads file at path into dst, up to max_bytes; returns size read.
; int fat32_load_file(fat32_bpb_t* v, const char* path, void* dst,
;                     uint32_t max_bytes, uint32_t* out_size);


%endif
%define __INC_FAT32_SYS