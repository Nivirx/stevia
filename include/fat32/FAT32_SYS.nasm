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

; what a 'mount' should probably do at this point...
; - read VBR at partition_lba
; - validate FAT32 signatures
; - fill fat32_bpb_t and compute derived fields
; - read FSInfo (free count/next free)
; - ???
; int fat32_mount(uint32_t partition_lba, fat32_bpb_t* out);


;
; Accessors
;

; // Convert cluster -> LBA of first sector of that cluster
; uint64_t clus_to_lba(const fat32_bpb_t* v, uint32_t clus) {
;  return v->data_lba + (uint64_t)(clus - 2) * v->secs_per_clus;
; }

; FAT32 entry is 32-bits; only low 28 bits are meaningful.
;   EOC if (val & 0x0FFFFFFF) >= 0x0FFFFFF8.
;   bad if == 0x0FFFFFF7.
;   free if == 0x00000000.

; // Read 32-bit FAT entry for cluster n
; int fat32_read_fat(const fat32_bpb_t* v, uint32_t clus, uint32_t* out);

; // Walk to next cluster in chain (validates EOC/bad)
; int fat32_next_clus(const fat32_bpb_t* v, uint32_t clus, uint32_t* out_next);

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