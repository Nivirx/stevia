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
    sub sp, 0x6

.calc_active_part:
    mov bx, [partition_offset_ptr]
    mov cx, word [ds:bx]                     
    mov ax, partition_table
    add ax, cx
    mov si, ax                                  ; si = offset to active partition
    
    mov ax, fat32_state
    mov di, ax

    mov eax, dword [si + PartEntry_t.lba_start]
    mov dword [di + FAT32_State_t.curr_drive_lba_32], eax

    mov ax, fat32_bpb
    mov si, ax

    mov ax, fat32_ebpb
    mov bx, ax
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
    ; BPB Info
    ; BS_OEMName = ident_8
    ; BPB_BytsPerSec = bytes_per_sector_16
    ; BPB_SecPerClus = sectors_per_cluster_8
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
    .load_first_dir:
        mov eax, dword [fat32_state + FAT32_State_t.curr_dir_cluster_32]
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
        
        mov eax, dword [fat32_state + FAT32_State_t.curr_dir_cluster_32]
        push dword eax
        call NextCluster        ; uint32_t NextCluster(uint32_t active_cluster);
        add sp, 0x4

        cmp eax, 0x0fff_fff7
        ;je SearchFATDIR.bad_cluster
        jb SearchFATDIR.load_next_dir_next_OK
        ERROR STAGE2_FAT32_END_OF_CHAIN

    .load_next_dir_next_OK:
        ; load 512 bytes of directory entries from data sector
        mov eax, [fat32_state + FAT32_State_t.curr_dir_cluster_32]
        push dword eax          ; cluster

        mov ax, dir_buffer    
        push ax                 ; offset

        xor ax, ax
        push ax                 ; segment
        call ReadFATCluster     ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
        sub sp, 0x8
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
            mov di, BootTarget_str          ; current memory location (8.3 name is at offset 0)
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
    mov edx, dword [bp + 4]
    mov si, fat32_nc_data                        ; instead of push/pop and moving the data back
    mov di, fat32_bpb                            ; load si & di then use xchg
.calc_offset:
; fat_offset = active_cluster * 4
    mov eax, 4
    mul edx                                      
    jc NextCluster.error_cfdivz
    mov dword [si + FAT32_NextClusterData_t.fat_offset], eax        ; move lower 32 bits to fat offset

.calc_fat_sector:
; fat_sector = first_fat_sector + (fat_offset / sector_size)
; entry_offset = fat_offset % sector_size
    mov edx, 0xffff_0000
    and edx, eax
    shr edx, 16

    xchg si, di         ; switch to fat32_bpb in si
    mov cx, word [si + FAT32_bpb_t.bytes_per_sector_word]
    xchg si, di

    cmp edx, 0
    je NextCluster.error_cfdivz
    div cx              ; DX:AX / cx = fat_sector - first_fat_sector in AX
                        ; DX = remainder (fat_offset mod sector_size)
    
    mov ecx, 0x0000_ffff
    and edx, ecx
    mov dword [si + FAT32_NextClusterData_t.entry_offset], edx

    xchg si, di         ; switch to fat32_bpb in si
    mov ecx, dword [si + FAT32_State_t.first_fat_sector_32]
    xchg si, di

    mov edx, 0x0000ffff
    and eax, edx
    add eax, ecx        ; fat_sector + first_fat_sector
    mov dword [si + FAT32_NextClusterData_t.fat_sector], eax
.load_fat_table:
    mov bx, word [boot_drive_ptr]
    movzx ax, byte [ds:bx]
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
    ; BUG: something about this function causes this BIOS call to come back wrong
    ; somehow the drive number is being read in wrong, but I can't find evidence of it being modified or writen
    ; to memory incorrectly.
    ; 00007109574i[BIOS  ] Booting from 0000:7c00
    ; 00007123227i[CPU0  ] [7123227] Stopped on MAGIC BREAKPOINT
    ; (0) Magic breakpoint
    ; Next at t=7123227
    ; (0) [0x000000000502] 0000:0502 (unk. ctxt): cli                       ; fa
    ; <bochs:2> c
    ; 00007252963i[BIOS  ] int13_diskette: unsupported AH=42
    ; 00007253071i[CPU0  ] [7253071] Stopped on MAGIC BREAKPOINT
    ; (0) Magic breakpoint
    ; Next at t=7253071
    ; (0) [0x00000000095c] 0000:095c (unk. ctxt): mov al, 0x47              ; b047
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

; uint32_t ClusterToLBA(uint32_t cluster)
ALIGN 4, db 0x90
ClusterToLBA:
    __CDECL16_ENTRY
.func:
    mov dword eax, [bp + 4]
    sub eax, 2
    movzx edx, byte [fat32_bpb + FAT32_bpb_t.sectors_per_cluster_byte]
    mul edx
    jc ClusterToLBA.error
    add eax, dword [fat32_state + FAT32_State_t.first_data_sector_32]
    ; eax contains the LBA now
.endp:
    __CDECL16_EXIT
    ret
.error:
    ERROR STAGE2_FAT32_CLS2LBA_CF

; bp - 2 - byte boot_drive
; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
ALIGN 4, db 0x90
ReadFATCluster:
    __CDECL16_ENTRY
.func:
    print_string ReadFATCluster_info

    mov bx, word [boot_drive_ptr]
    movzx ax, byte [ds:bx]
    push ax

    mov ax, 0x1                         ; count = 1
    push ax

    mov eax, dword [bp + 8]
    push dword eax
    call ClusterToLBA
    add sp, 0x4
    ; eax contains the LBA now
    push dword eax                      ; lba = ClusterToLBA(..)

    mov ax, fat_buffer                  ; offset = fat_buffer (in mem.inc)
    push ax

    xor ax, ax                          ; segment = 0x0000, our buffer is in the first 64 KiB
    push ax
; uint8_t read_stage2_raw(uint16_t buf_segment, uint16_t buf_offset, 
;                         uint32_t lba,
;                         uint16_t count, uint16_t drive_num)
    call read_disk_raw  
    add sp, 0xC
.endp:
    __CDECL16_EXIT
    ret

%endif
%define __INC_FAT32_SYS