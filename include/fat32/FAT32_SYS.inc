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

; Clobbers: eax, edx
; returns: none
ALIGN 4, db 0x90
InitFATDriver:
    __CDECL16_ENTRY
.func:
    mov ax, FAT32_State_t_size
    push ax                     ; length of fat32_state structure
    xor ax, ax
    push ax                     ; init fat32_state with zero
    mov ax, fat32_state
    push ax                     ; address of structure
    call kmemset
    add sp, 0x6

.calc_active_part:
    mov ax, word [partition_offset]            
    mov di, fat32_state

    mov dx, partition_table
    add dx, ax                                                           ; dx points to the partition that was booted from
    mov bx, dx                                                           ; set bx, should point at our partition

    mov eax, dword [bx + PartEntry_t.lba_start]
    mov dword [di + FAT32_State_t.curr_drive_lba_32], eax

    mov si, fat32_bpb
    mov bx, fat32_ebpb
.calc_first_fat:
    movzx eax,  word [si + FAT32_bpb_t.reserved_sectors_word]        ; first fat from start of partition
    add   eax, dword [di + FAT32_State_t.curr_drive_lba_32]          ; calculate offset from start of drive
    jc InitFATDriver.error

    mov dword [di + FAT32_State_t.first_fat_sector_32], eax
.calc_total_fat:
    mov edx,  dword [bx + FAT32_ebpb_t.FATSz_dword]
    movzx eax, byte [si + FAT32_bpb_t.fat_count_byte]
    mul edx                                                                ; result in EDX:EAX, CF set on > 32bit return value
    jc InitFATDriver.error                                                 ; as a catch for unhandled overflow, just error if value is greater than 32bits

    mov dword [di + FAT32_State_t.fat_size_32], eax
.calc_first_data:
    mov edx, dword [di + FAT32_State_t.first_fat_sector_32]
    add eax, edx
    jc InitFATDriver.error

    mov dword [di + FAT32_State_t.first_data_sector_32], eax
.set_first_dir:
    mov eax, dword [bx + FAT32_ebpb_t.root_clus_dword]
    mov dword [di + FAT32_State_t.curr_dir_cluster_32], eax
.endp:
    __CDECL16_EXIT
    ret
.error:
    ERROR STAGE2_FAT32_INIT_CF

ALIGN 4, db 0x90
FSInfoPrinter:
    __CDECL16_ENTRY
.func:
    ;info we want to print to validate we are loading stuff from the disk correctly
    ; boot_drive # (i.e 0x80)
    ; active/bootable partition info (lba_start, lba_length, part_type)
    ; BPB Info: BS_OEMName = ident_8, BPB_BytsPerSec = bytes_per_sector_16, BPB_SecPerClus = sectors_per_cluster_8
    ; eBPB info: FATSz_dword, volid, vol label
    ; 
    ; print entire FAT32 state
    ;
.endp:
    __CDECL16_EXIT
    ret
.error:
    ERROR STAGE2_ERROR_INFOPRINTER

; this involves using the low memory buffer for the bios call and moving the file sector by sector to high memory
;
; SFN is a 8.3 file name, all uppercase, and padded with spaces
; do not add a null byte to the end of the string
; eg. kernel.bin == "KERNEL  BIN"
;
; returns first cluster of file if found
; halts/errors if file is not found
; uint32_t SearchFATDIR(uint8_t* SFN);
ALIGN 4, db 0x90
SearchFATDIR:
    __CDECL16_ENTRY
.file_lookup:
    print_string SearchFATDIR_info
    mov bx, fat32_state
    .load_first_dir:
        mov eax, dword [bx + FAT32_State_t.curr_dir_cluster_32]
        push dword eax          ; cluster
        mov ax, dir_buffer
        push ax                 ; offset
        xor ax, ax
        push ax                 ; segment
        call ReadFATCluster     ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
        add sp, 0x8

        mov si, dir_buffer
        jmp SearchFATDIR.empty_dir_entry

    .load_next_dir:
        ; if eax >= 0x0FFFFFF8 then there are no more clusters (end of chain)
        ; if eax == 0x0FFFFFF7 then this is a cluster that is marked as bad
        
        mov eax, dword [bx + FAT32_State_t.curr_dir_cluster_32]
        push dword eax
        call NextCluster        ; uint32_t NextCluster(uint32_t active_cluster);
        add sp, 0x4

        cmp eax, 0x0fff_fff7
        ;je SearchFATDIR.bad_cluster            ; TODO: Implement Bad cluster checks
        jb SearchFATDIR.load_next_dir_next_OK
        ERROR STAGE2_FAT32_END_OF_CHAIN

    .load_next_dir_next_OK:
        ; load 512 bytes of directory entries from data sector
        mov eax, [bx + FAT32_State_t.curr_dir_cluster_32]
        push dword eax          ; cluster

        mov ax, dir_buffer    
        push ax                 ; offset

        xor ax, ax
        push ax                 ; segment
        call ReadFATCluster     ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
        add sp, 0x8
    .empty_dir_entry:
        ; check for 0x0 in first byte, if true then there are no more files
        ; if true we did not find the file, we should error here
        cmp byte [si], 0
        jne SearchFATDIR.unused_dir_entry
        ERROR STAGE2_FAT32_NO_FILE

    .unused_dir_entry:
        ; check for 0xe5 and 0x05 in first byte, if true then this entry is unused, but it is not the last entry.
        cmp byte [si], 0xe5
        je SearchFATDIR.next_entry

        cmp byte [si], 0x05
        je SearchFATDIR.next_entry
        jmp SearchFATDIR.parse_dir

    .next_entry:
        ; increment offset by 32 bytes to read the next entry in this set of dir entries
        ; if we are at the end of the buffer, then load the next buffer
        add si, 0x20        ; 32 bytes

        mov ax, dir_buffer
        add ax, 0x1FF       ; 512 - 1 bytes
        cmp si, ax
        jae SearchFATDIR.load_next_dir
        jmp SearchFATDIR.empty_dir_entry

    ; TODO: move this to a seperate string search function
    .parse_dir:
        print_string MaybeFound_Boot_info
        .lfn_check:
            ; check for ATTR_READ_ONLY | ATTR_HIDDEN | ATTR_SYSTEM | ATTR_VOLUME_ID  (0x0F) in offset 11
            ; TODO: going to skip LFN for now, since all valid volumes will have SFN's
            cmp byte [si+11], 0x0F
            je SearchFATDIR.next_entry
        .sfn_file_name_check:
            push si
            push di

            mov cx, 0xA                     ; max of 11 filename length of 11 characters
            ; si points to the start of the current directory entry
            mov di, BootTarget              ; current memory location (8.3 name is at offset 0)
            repe cmpsb                      ; compare the strings

            pop di
            pop si
            jne SearchFATDIR.next_entry

            .sfn_entry_found:
            mov ax, [si + FAT32_SFN_t.cluster_16_high]
            shl eax, 16
            mov ax, [si + FAT32_SFN_t.cluster_16_low]
            ; eax == first cluster of file
.endp:
    __CDECL16_EXIT
    ret

; BUG: this function needs review
; uint32_t NextCluster(uint32_t active_cluster);
; if eax >= 0x0FFFFFF8 then there are no more clusters (end of chain)
; if eax == 0x0FFFFFF7 then this is a cluster that is marked as bad
ALIGN 4, db 0x90
NextCluster:
    __CDECL16_ENTRY
.func:
    print_string NextFATCluster_info

    mov ax, FAT32_NextClusterData_t_size
    push ax                     ; length
    xor ax, ax
    push ax                     ; init with zero
    mov ax, fat32_nc_data
    push ax                     ; address of structure
    call kmemset
    add sp, 0x6

    mov edx, dword [bp + 4]
    mov si, fat32_nc_data                        ; instead of push/pop and moving the data back
    mov di, fat32_bpb                            ; load si & di then use xchg
    mov bx, fat32_state
.calc_offset:
; fat_offset = active_cluster * 4
    mov eax, 4
    mul edx                                      
    jc NextCluster.error_cfdivz
    mov dword [si + FAT32_NextClusterData_t.fat_offset], eax        ; move lower 32 bits to fat offset
.calc_fat_sector:
; fat_sector = first_fat_sector + (fat_offset / sector_size)
; entry_offset = fat_offset % sector_size
    xor edx, edx
    movzx ecx, word [di + FAT32_bpb_t.bytes_per_sector_word]
    cmp eax, 0
    je NextCluster.error_cfdivz
    div ecx              ; eDX:eAX / eCX = fat_sector - first_fat_sector in eAX
                        ; eDX = remainder (fat_offset mod sector_size)

    mov dword [si + FAT32_NextClusterData_t.entry_offset], edx
    mov ecx, dword [bx + FAT32_State_t.first_fat_sector_32]
    add eax, ecx        ; fat_sector + first_fat_sector
    mov dword [si + FAT32_NextClusterData_t.fat_sector], eax
.load_fat_table:
    xor ax, ax
    mov al, byte [boot_drive]
    push ax

    mov ax, 0x1
    push ax

    ; load correct fat
    mov eax, dword [si + FAT32_NextClusterData_t.fat_sector]
    push dword eax

    mov ax, fat_buffer
    push ax

    xor ax, ax
    push ax
    call read_disk_raw  
    add sp, 0xC
    ; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
;                         uint32_t lba,
;                         uint16_t count, uint16_t drive_num)
.read_cluster:
; next_cluster = fat_buffer[entry_offset]
    mov ebx, dword [si + FAT32_NextClusterData_t.entry_offset]
    mov si, fat_buffer
    mov eax, dword [bx+si+0]
.endp:
    __CDECL16_EXIT
    ret
.error_cfdivz:
    ERROR STAGE2_FAT32_NCLUS_CFDIVZ

; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
ALIGN 4, db 0x90
ReadFATCluster:
    __CDECL16_ENTRY
.func:
    print_string ReadFATCluster_info

    mov bx, fat32_bpb
    mov si, fat32_ebpb

    xor ax, ax
    mov al, byte [boot_drive]
    push ax

    mov ax, 0x1                         ; count = 1
    push ax

    mov eax, dword [bp + 8]             ; cluster
    sub eax, 2
    movzx edx, byte [bx + FAT32_bpb_t.sectors_per_cluster_byte]
    mul edx                             ; result in eax, error on carry 
    jc ReadFATCluster.error
    add eax, dword [si + FAT32_State_t.first_data_sector_32]
    jc ReadFATCluster.error
    ; eax contains the LBA now
    push dword eax                      ; lba = ClusterToLBA(..)

    mov ax, word [bp + 6]               ; offset
    push ax

    mov ax, word [bp + 4]
    push ax
    ; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, 
    ;                         uint32_t lba,
    ;                         uint16_t count, uint16_t drive_num)
    call read_disk_raw  
    add sp, 0xC
.endp:
    __CDECL16_EXIT
    ret
.error:
    ERROR STAGE2_FAT32_CLS2LBA_CF

%endif
%define __INC_FAT32_SYS