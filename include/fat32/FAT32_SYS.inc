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

%ifndef __INC_FAT32_SYS

%include "partition_table.inc"
%include "fat32/bpb_offset_bx.inc"
%include "fat32/fat32_structures.inc"

ALIGN 4, db 0x90
InitFATDriver:
    __CDECL16_ENTRY
.func:
    mov ax, fat32_state_SIZE
    push ax                     ; length of fat32_state structure
    xor ax, ax
    push ax                     ; init fat32_state with zero
    mov ax, fat32_state
    push ax                     ; address of structure
    call kmemset
    sub sp, 0x6

.calc_active_part:
    mov bx, [partition_offset_ptr]
    mov ax, [bx + 0]                     

    mov dx, partition_table
    add dx, ax                                                           ; dx points to the partition that was booted from
    mov bx, dx                                                           ; set bx, should point at our partition

    mov eax, dword [bx + PartEntry_t.lba_start]
    mov dword [fat32_state + FAT32_State_t.active_drive_lba_32], eax

.calc_first_fat:
    movzx eax,  word [fat32_bpb + FAT32_bpb_t.reserved_sectors_16]        ; first fat from start of partition
    add   eax, dword [fat32_state + FAT32_State_t.active_drive_lba_32]    ; calculate offset from start of drive

    mov dword [fat32_state + FAT32_State_t.first_fat_sector_32], eax
.calc_total_fat:
    mov ebx,  dword [fat32_ebpb + FAT32_ebpb_t.sectors_per_fat_32]
    movzx eax, byte [fat32_bpb + FAT32_bpb_t.fat_count_8]
    mul ebx                                                                ; result in EDX:EAX, CF set on > 32bit return value
    jc InitFATDriver.error                                                 ; as a catch for unhandled overflow, just error if value is greater than 32bits

    mov dword [fat32_state + FAT32_State_t.fat_size_32], eax
.calc_first_data:
    mov edx, dword [fat32_state + FAT32_State_t.first_fat_sector_32]
    add eax, edx

    mov dword [fat32_state + FAT32_State_t.first_data_sector_32], eax
.get_first_root_dir:
    ; TODO
    jmp InitFATDriver.endp
.error:
    ERROR STAGE2_FAT32_INIT_ERROR
.endp:
    __CDECL16_EXIT
    ret

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
    .load_first_dir:
        mov eax, [fat32_ebpb + FAT32_ebpb_t.root_dir_cluster_32]
        mov dword [fat32_state + FAT32_State_t.active_dir_cluster_32], eax

        __CDECL16_CALLER_SAVE

        push dword eax          ; cluster
        lea ax, [dir_buffer]
        push ax                 ; offset
        xor ax, ax
        push ax                 ; segment
        call ReadFATCluster     ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
        add sp, 0x8

        __CDECL16_CALLER_RESTORE

        lea si, [dir_buffer]
        jmp SearchFATDIR.empty_dir_entry

    .load_next_dir:
        ; if eax >= 0x0FFFFFF8 then there are no more clusters (end of chain)
        ; if eax == 0x0FFFFFF7 then this is a cluster that is marked as bad
        __CDECL16_CALLER_SAVE
        mov eax, dword [fat32_state + FAT32_State_t.active_dir_cluster_32]
        push dword eax
        call NextCluster        ; uint32_t NextCluster(uint32_t active_cluster);
        add sp, 0x4
        __CDECL16_CALLER_RESTORE

        cmp eax, 0x0fff_fff7
        jb SearchFATDIR.load_next_dir_next_OK
        ERROR STAGE2_FAT32_END_OF_CHAIN

    .load_next_dir_next_OK:
        ; load 512 bytes of directory entries from data sector
        __CDECL16_CALLER_SAVE
        mov eax, [fat32_state + FAT32_State_t.active_dir_cluster_32]
        push dword eax          ; cluster

        lea ax, [dir_buffer]    
        push ax                 ; offset

        xor ax, ax
        push ax                 ; segment
        call ReadFATCluster     ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
        sub sp, 0x8
        __CDECL16_CALLER_RESTORE

        lea si, [dir_buffer]
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

        lea ax, [dir_buffer]
        add ax, 0x1FF       ; 512 - 1 bytes
        cmp si, ax
        jae SearchFATDIR.load_next_dir
        jmp SearchFATDIR.empty_dir_entry

    .parse_dir:
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
            lea di, [BootTarget_str]        ; current memory location (8.3 name is at offset 0)
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
    mov bx, [boot_drive_ptr]
    mov ax, [bx + 0]
    mov bx, ax
    movzx ax, byte [bx + 0]                      ; 67h override would probably work but this is the 16bit way to do it
    mov byte [bp - 2], al                        ; save boot drive as a local for easy access

    mov dword edx, [bp + 4]
    mov si, fat32_nc_data
.calc_offset:
; fat_offset = active_cluster * 4
    mov eax, 4
    mul edx
    mov dword [si + FAT32_NextClusterData_t.fat_offset], eax

.calc_fat_sector:
; fat_sector = first_fat_sector + (fat_offset / sector_size)
; entry_offset = fat_offset % sector_size
    mov edx, 0xffff_0000
    and edx, eax
    shr edx, 16

    push si
    mov si, fat32_bpb
    mov cx, word [si + FAT32_bpb_t.bytes_per_sector_16]
    pop si

    div cx              ; DX:AX / cx = fat_sector - first_fat_sector in AX
                        ; DX = remainder (fat_offset mod sector_size)
    
    mov ecx, 0x0000_ffff
    and edx, ecx
    mov dword [si + FAT32_NextClusterData_t.entry_offset], edx

    push si
    mov si, fat32_state
    mov ecx, dword [si + FAT32_State_t.first_fat_sector_32]
    pop si

    mov edx, 0x0000ffff
    and eax, edx
    add eax, ecx        ; fat_sector + first_fat_sector
    mov dword [si + FAT32_NextClusterData_t.fat_sector], eax
.load_fat_table:
    __CDECL16_CALLER_SAVE
    movzx ax, [bp - 2]
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
    __CDECL16_CALLER_RESTORE
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

; uint32_t ClusterToLBA(uint32_t cluster)
ALIGN 4, db 0x90
ClusterToLBA:
    __CDECL16_ENTRY
.func:
    mov dword eax, [bp + 4]
    sub eax, 2
    movzx edx, byte [fat32_bpb + FAT32_bpb_t.sectors_per_cluster_8]
    mul edx
    add eax, dword [fat32_state + FAT32_State_t.first_data_sector_32]
    ; eax contains the LBA now
.endp:
    __CDECL16_EXIT
    ret

; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
ALIGN 4, db 0x90
ReadFATCluster:
    __CDECL16_ENTRY
.func:
    __CDECL16_CALLER_SAVE
    mov bx, [boot_drive_ptr]
    mov ax, [bx + 0]
    mov byte [bp - 2], al                        ; save boot drive as a local for easy access

    mov dword eax, [bp + 8]
    push dword eax
    call ClusterToLBA    ; uint32_t ClusterToLBA(uint32_t cluster)
    add sp, 0x4         ; eax == LBA
    mov dword [bp - 6], eax
    __CDECL16_CALLER_RESTORE

    __CDECL16_CALLER_SAVE
    movzx ax, [bp - 2]
    push ax

    mov ax, 0x1
    push ax

    mov eax, dword [bp + 6]
    push dword eax

    mov ax, fat_buffer
    push ax

    xor ax, ax
    push ax
; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
;                         uint32_t lba,
;                         uint16_t count, uint16_t drive_num)
    call read_disk_raw  
    add sp, 0xC
    __CDECL16_CALLER_RESTORE
.endp:
    __CDECL16_EXIT
    ret

%endif
%define __INC_FAT32_SYS