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

[BITS 16]
[ORG 0X7E00]
[CPU KATMAI]
jmp short init
nop

; Unless noted otherwise, functions will be the standard x86 cdecl used in GCC

; In cdecl, subroutine arguments are passed on the stack. 
; Integer values and memory addresses are returned in the EAX register,
; floating point values in the ST0 x87 register. 
; Registers EAX, ECX, and EDX are caller-saved, 
; and the rest are callee-saved. 
; The x87 floating point registers ST0 to ST7 must be empty (popped or freed) 
; when calling a new function, and ST1 to ST7 must be empty on exiting a function. 
; ST0 must also be empty when not used for returning a value.

; boot drive in dl
; active partition offset in si
%include "entry.inc"
init:
    cli                         ; We do not want to be interrupted

    xor ax, ax                  ; 0 AX
    mov ds, ax                  ; Set segment registers to 0
    mov es, ax                  ; *
    mov fs, ax                  ; *
    mov gs, ax                  ; *

    mov ss, ax                  ; Set Stack Segment to 0
    mov sp, STACK_START         ; Set Stack Pointer
    mov bp, sp

    jmp 0:main

%include "config.inc"
%include "errors.inc"
%include "memory.inc"

%include "partition_table.inc"

%include "fat32/bpb.inc"
%include "fat32/fat32_structures.inc"

main:
    sti

    mov byte [fat32_ebpb + FAT32_ebpb_t.drive_number_8], dl
    mov word [partition_offset], si

    call SetTextMode
    call disable_cursor

    mov eax, dword [STAGE2_SIG]
    cmp eax, 0xDEADBEEF
    je main.signature_present

    ERROR STAGE2_SIGNATURE_MISSING

.signature_present:
    mov ax, 0xDEAD
    push ax
    mov ax, 0xBEEF
    push ax             ; mark top of stack for debuging

    push bp
    mov bp, sp
    lea ax, [HelloPrompt_cstr]
    push ax
    call PrintString
    leave

    ; enable A20 gate
    push bp
    mov bp, sp
    call EnableA20
    lea ax, [A20_Enabled_cstr]
    push ax
    call PrintString
    leave

    ; get system memory map
    push bp
    mov bp, sp
    call GetMemoryMap
    lea ax, [MemoryMap_OK_cstr]
    push ax
    call PrintString
    leave

    ; enter unreal mode
    push bp
    mov bp, sp
    call EnterUnrealMode
    lea ax, [UnrealMode_OK_cstr]
    push ax
    call PrintString
    leave

    push bp
    mov bp, sp
    call InitFATDriver
    leave

    ;
    ; Find first cluster of bootable file
    ;
    push bp
    mov bp, sp
    call SearchFATDIR
    leave
    PUSH_DWORD_EAX      ; save return value of function

    push bp
    mov bp, sp
    lea ax, [FileFound_OK_cstr]
    push ax
    call PrintString
    leave

    POP_DWORD_EAX       ; return value of SearchFATDIR
    push bp
    mov bp, sp
    PUSH_DWORD_EAX
    call PrintDWORD
    leave

    push bp
    mov bp, sp
    lea ax, [NewLine_cstr]
    push ax
    call PrintString
    leave
    
    ; TODO

    ; read elf information and place the sections in the file at the correct location
    ; TODO

    ; jump to entry point
    ; TODO

hcf:
    hlt
    jmp short hcf

; ###############
;
; FAT32 Driver
;
; ###############


InitFATDriver:
    push bx

    xor eax, eax
    mov dword [fat32_state + FAT32_State_t.active_cluster_32], eax
    mov dword [fat32_state + FAT32_State_t.active_FAT_cluster_32], eax
    mov dword [fat32_state + FAT32_State_t.first_root_dir_sector_32], eax
    mov dword [fat32_state + FAT32_State_t.active_dir_cluster_32], eax

.calc_active_part:
    mov ax, word [partition_offset]
    mov bx, partition_table
    add bx, ax                                                           ; bx points to the partition that was booted from

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
    pop bx
    ret

; load a file to the high memory buffer for the elf parser
; this involves using the low memory buffer for the bios call and moving the file sector by sector to high memory
;
; SFN is a 8.3 file name, all uppercase, and padded with spaces
; do not add a null byte to the end of the string
; eg. kernel.bin == "KERNEL  BIN"
;
; returns first cluster of file if found
; halts/errors if file is not found
; uint32_t SearchFATDIR(uint8_t* SFN);
SearchFATDIR:
    push di
    push si
    push bx

.file_lookup:
    xor ecx, ecx
    xor edx, edx
    .load_first_dir:
        push bp
        mov bp, sp
        ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)

        mov eax, [fat32_ebpb + FAT32_ebpb_t.root_dir_cluster_32]
        mov dword [fat32_state + FAT32_State_t.active_dir_cluster_32], eax
        PUSH_DWORD_EAX
        lea ax, [dir_buffer]
        push ax                 ; offset
        xor ax, ax
        push ax                 ; segment

        call ReadFATCluster
        leave
        lea si, [dir_buffer]
        jmp SearchFATDIR.empty_dir_entry

    .load_next_dir:
        ; uint32_t NextCluster(uint32_t active_cluster);
        ; if eax >= 0x0FFFFFF8 then there are no more clusters (end of chain)
        ; if eax == 0x0FFFFFF7 then this is a cluster that is marked as bad
        push bp
        mov bp, sp

        mov eax, dword [fat32_state + FAT32_State_t.active_dir_cluster_32]
        PUSH_DWORD_EAX
        call NextCluster
        leave

        cmp eax, 0x0fff_fff7
        jb SearchFATDIR.load_next_dir_next_OK
        ERROR STAGE2_FAT32_END_OF_CHAIN

    .load_next_dir_next_OK:
        ; load 512 bytes of directory entries from data sector
        push bp
        mov bp, sp
        ; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)

        mov eax, [fat32_state + FAT32_State_t.active_dir_cluster_32]
        PUSH_DWORD_EAX
        lea ax, [dir_buffer]
        push ax                 ; offset
        xor ax, ax
        push ax                 ; segment

        call ReadFATCluster
        leave

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
    pop bx
    pop si
    pop di
    ret


; uint32_t NextCluster(uint32_t active_cluster);
; if eax >= 0x0FFFFFF8 then there are no more clusters (end of chain)
; if eax == 0x0FFFFFF7 then this is a cluster that is marked as bad
NextCluster:
    push si
    push di
    push bx

    MOV_DWORD_EAX 2
    mov edx, eax
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
; load correct fat 
    push bp
    mov bp, sp
; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, uint16_t lower_lower_lba, uint16_t upper_lower_lba)

    ror eax, 16
    push ax
    ror eax, 16
    push ax

    mov ax, fat_buffer
    push ax

    xor ax, ax
    push ax

    call read_disk_raw  ; read_disk_raw(0, fat_buffer, 31:16 fat_sector, 15:0 fat_sector)
    leave

.read_cluster:
; next_cluster = fat_buffer[entry_offset]
    mov ebx, dword [si + FAT32_NextClusterData_t.entry_offset]
    mov si, fat_buffer
    mov eax, dword [bx+si+0]
.endp:
    pop bx
    pop di
    pop si
    ret

; uint32_t ClusterToLBA(uint32_t cluster)
ClusterToLBA:
    push si
    push di

    MOV_DWORD_EAX 2
    sub eax, 2
    movzx edx, byte [fat32_bpb + FAT32_bpb_t.sectors_per_cluster_8]
    mul edx
    add eax, dword [fat32_state + FAT32_State_t.first_data_sector_32]
    ; eax contains the LBA now
.endp:
    pop di
    pop si
    ret

; uint8_t ReadFATCluster(uint16_t seg, uint16_t offset, uint32_t cluster)
ReadFATCluster:
    push si
    push di

    ; cluster to LBA

    MOV_DWORD_EAX 2

    push bp
    mov bp, sp
    ; uint32_t ClusterToLBA(uint32_t cluster)
    PUSH_DWORD_EAX
    call ClusterToLBA
    leave ; eax == LBA

    mov dx, [bp-8] ; seg
    shl edx, 16
    mov dx, [bp-6] ; offset

    

    ; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, uint16_t lower_lower_lba, uint16_t upper_lower_lba)
    push bp
    mov bp, sp

    ror eax, 16
    push ax
    ror eax, 16
    push ax

    push dx
    ror edx, 16
    push dx

    ;DEBUG_HCF

    call read_disk_raw
    leave

.endp:
    pop di
    pop si
    ret

; ###############
;
; BIOS functions
;
; ###############


; Wrapper for AH=0x42 INT13h (Extended Read)
;
; BIOS call details
; AH = 42h
; DL = drive number
; DS:SI -> disk address packet
;
; Return:
; CF clear if successful
; AH = 00h
; CF set on error
; AH = error code
; disk address packet's block count field set to number of blocks
; successfully transferred
;
;
; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, uint16_t lower_lower_lba, uint16_t upper_lower_lba)
; bp-0 = call frame
; bp-2 = upper_lower_lba
; bp-4 = lower_lower_lba
; bp-6 = offset
; bp-8 = segment
; bp-10 = ret ptr
read_disk_raw:
    push si

    ; uint8_t* kmemset(void* dest, uint8_t val, size_t len);
    push bp
    mov bp, sp

    mov ax, 0x10
    push ax             ; len = 16 bytes
    xor ax, ax
    push ax             ; val = 0
    mov ax, lba_packet
    push ax             ; dest = lba_packet address

    call kmemset
    leave

    mov byte [lba_packet + LBAPkt_t.size], 0x10
    mov word [lba_packet + LBAPkt_t.xfer_size], 0x0001

    mov ax, [bp-2]
    shl eax, 16
    mov ax, [bp-4]
    mov dword [lba_packet + LBAPkt_t.lower_lba], eax

    mov ax, [bp-6]
    mov word [lba_packet + LBAPkt_t.offset], ax

    mov ax, [bp-8]
    mov word [lba_packet + LBAPkt_t.segment], ax

    mov si, lba_packet
    mov ah, 0x42
    mov dl, byte [fat32_ebpb + FAT32_ebpb_t.drive_number_8]
    int 0x13
    jnc read_disk_raw.endf
    ERROR STAGE2_MBR_DISK_READ_ERROR

.endf:
    pop si
    ret

; Prints a C-Style string (null terminated) using BIOS vga teletype call
; void PrintString(char* buf)
PrintString:
    push si
    push di
    push bx

    mov di, [bp - 2]    ; first arg is char* 
.str_len:

    xor cx, cx         ; ECX = 0
    not cx             ; ECX = -1 == 0xFFFF
    xor ax, ax         ; search for al = 0x0

    cld
    repne scasb        ; deincrement cx while searching for al

    not cx             ; the inverse of a neg number = abs(x) - 1
    dec cx             ; CX contains the length of the string - nul byte at end

.print:
    mov si, [bp - 2]            ; source string
.print_L0:
    push bp
    mov bp, sp

    movzx ax, byte [si]
    push ax

    call PrintCharacter
    leave

    inc si
    dec cx

    jcxz PrintString.endp
    jmp PrintString.print_L0    ; Fetch next character from string
.endp:
    pop bx
    pop di
    pop si
    ret                         ; Return from procedure

; Prints a single character
; void PrintCharacter(char c);
PrintCharacter:
    push si
    push di
    push bx

    mov ax, [bp-2] ; c
    mov dx, 0x00ff
    and ax, dx

    mov ah, 0x0E                ; INT 0x10, AH=0x0E call
    mov bx, 0x0007              ; BH = page no. BL =Text attribute 0x07 is lightgrey font on black background
    int 0x10                    ; call video interrupt

.endp:
    pop bx
    pop di
    pop si
    ret

; prints the hex representation of of val_upper:val_lower (4 byte value)
; void PrintDWORD(uint16_t val_upper, uint16_t val_lower);
PrintDWORD:
    push si
    lea si, [IntToHex_table]

    push bx
    mov ebx, 16     ; base-16

    mov ax, [bp-4] ; val_upper
    shl eax, 16
    mov ax, [bp-2] ; val_lower

    xor edx, edx
    xor cx, cx
.next_digit:
    div ebx            ; dividend in edx:eax -> quotient in eax, remainder in edx
    push dx            ; save remainder
    inc cx

    xor dx, dx
    test eax, eax
    jnz PrintDWORD.next_digit

.zero_pad:
    cmp cx, 0x0008
    je PrintDWORD.print_stack
    xor ax, ax
    push ax
    inc cx
    jmp PrintDWORD.zero_pad

.print_stack:
    pop bx
    dec cx
    push cx

    push bp
    mov bp, sp
    movzx ax, byte [bx+si+0]    ; bx = index into Hex lookup table
    push ax
    call PrintCharacter
    leave

    pop cx

    jcxz PrintDWORD.endp
    jmp PrintDWORD.print_stack

.endp:
    pop bx
    pop si
    ret

; Sets output to 80x25 16 color text mode via BIOS call
; also clears screen
; void SetTextMode(void)
SetTextMode:
    xor ah, ah                  ; Set Video mode BIOS function
    mov al, 0x02                ; 16 color 80x25 Text mode
    int 0x10                    ; Call video interrupt

    mov ah, 0x05                ; Select active display page BIOS function
    xor al, al                  ; page 0
    int 0x10                    ; call video interrupt
.endp:
    ret

; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		Low 32 Bits of Base Address
;	4	    BaseAddrHigh	High 32 Bits of Base Address
;	8	    LengthLow		Low 32 Bits of Length in Bytes
;	12	    LengthHigh		High 32 Bits of Length in Bytes
;	16	    Type		Address type of  this range.
; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		Low 32 Bits of Base Address
;	4	    BaseAddrHigh	High 32 Bits of Base Address
;	8	    LengthLow		Low 32 Bits of Length in Bytes
;	12	    LengthHigh		High 32 Bits of Length in Bytes
;	16	    Type		Address type of  this range.
; Input:
;
;	EAX	Function Code	E820h
;	EBX	Continuation	Contains the "continuation value" to get the
;				next run of physical memory.  This is the
;				value returned by a previous call to this
;				routine.  If this is the first call, EBX
;				must contain zero.
;	ES:DI	Buffer Pointer	Pointer to an  Address Range Descriptor
;				structure which the BIOS is to fill in.
;	ECX	Buffer Size	The length in bytes of the structure passed
;				to the BIOS.  The BIOS will fill in at most
;				ECX bytes of the structure or however much
;				of the structure the BIOS implements.  The
;				minimum size which must be supported by both
;				the BIOS and the caller is 20 bytes.  Future
;				implementations may extend this structure.
;	EDX	Signature	'SMAP' -  Used by the BIOS to verify the
;				caller is requesting the system map
;				information to be returned in ES:DI.
;
; Output:
;
;	CF	Carry Flag	Non-Carry - indicates no error
;	EAX	Signature	'SMAP' - Signature to verify correct BIOS
;				revision.
;	ES:DI	Buffer Pointer	Returned Address Range Descriptor pointer.
;				Same value as on input.
;	ECX	Buffer Size	Number of bytes returned by the BIOS in the
;				address range descriptor.  The minimum size
;				structure returned by the BIOS is 20 bytes.
;	EBX	Continuation	Contains the continuation value to get the
;				next address descriptor.  The actual
;				significance of the continuation value is up
;				to the discretion of the BIOS.  The caller
;				must pass the continuation value unchanged
;				as input to the next iteration of the E820
;				call in order to get the next Address Range
;				Descriptor.  A return value of zero means that
;				this is the last descriptor
;
; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		Low 32 Bits of Base Address
;	4	    BaseAddrHigh	High 32 Bits of Base Address
;	8	    LengthLow		Low 32 Bits of Length in Bytes
;	12	    LengthHigh		High 32 Bits of Length in Bytes
;	16	    Type		Address type of  this range.
;
; The BaseAddrLow and BaseAddrHigh together are the 64 bit BaseAddress of this range.
; The BaseAddress is the physical address of the start of the range being specified.
;
; The LengthLow and LengthHigh together are the 64 bit Length of this range.
; The Length is the physical contiguous length in bytes of a range being specified.
;
; The Type field describes the usage of the described address range as defined in the table below.

; Value	       Pneumonic		           Description
;   1	    AddressRangeMemory	    This run is available RAM usable by the operating system.
;   2	    AddressRangeReserved	This run of addresses is in use or reserved by the system, and must not be used by the operating system.
; Other	    Undefined		        Undefined - Reserved for future use.
GetMemoryMap:
    push es         ; save segment registers

    push bx
    shr ebx, 16
    push bx         ; save ebx on a 16bit stack

    push cx
    shr ecx, 16
    push cx         ; save ecx on a 16bit stack
    ; end prolog
    mov dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries], 0

    mov eax, 0xE820                 ; select 0xE820 function
    mov ebx, 0x0000                 ; Continuation value

    mov dx, (BIOSMemoryMap >> 4)
    mov es, dx
    mov di, 0                      ; (BIOSMemoryMap >> 4):0 makes di an index into BIOSMemoryMap

    mov ecx, AddressRangeDescStruct_t_size
    mov edx, 0x534D4150                ; 'SMAP' magic

    int 0x15
    jnc GetMemoryMap.no_error
    ERROR STAGE2_MM_E820_NO_SUPPORT

.no_error:
    inc dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries]
    cmp eax, 0x534D4150
    jne GetMemoryMap.no_smap_returned

    cmp ecx, 20
    jb GetMemoryMap.nonstandard_e820

    cmp ebx, 0
    je GetMemoryMap.endp                  ; 0 in ebx means we have reached the end of memory ranges

    add di, AddressRangeDescStruct_t_size         ; increment di to next descriptor
    mov edx, eax                                ; 'SMAP' to edx
    mov eax, 0xE820                             ; select E820 function

    int 0x15
    jnc GetMemoryMap.no_error             ; carry indicates an error

.other_error:
    ERROR STAGE2_MM_E820_MISC_ERR
.nonstandard_e820:
    ERROR STAGE2_MM_E820_NONSTANDARD
.no_smap_returned:
    ERROR STAGE2_MM_E820_NO_SMAP

.endp:
    xor ebx, ebx
    xor ecx, ecx

    pop cx
    shl ecx, 16
    pop cx

    pop bx
    shl ebx, 16
    pop bx

    pop es

    ret

; ##############################
;
; SYSTEM CONFIGURATION FUNCTIONS
;
; ##############################

; disables blinking text mode cursor
disable_cursor:
	pushf
	push eax
	push edx
 
	mov dx, 0x3D4
	mov al, 0xA	    ; low cursor shape register
	out dx, al
 
	inc dx
	mov al, 0x20	; bits 6-7 unused, bit 5 disables the cursor, bits 0-4 control the cursor shape
	out dx, al
 
	pop edx
	pop eax
	popf
	ret

;
;NT 0x15 Function 2400 - Disable A20
;Returns:
;
;   CF = clear if success
;   AH = 0
;   CF = set on error
;   AH = status (01=keyboard controller is in secure mode, 0x86=function not supported)
;
;INT 0x15 Function 2401 - Enable A20
;Returns:
;
;   CF = clear if success
;   AH = 0
;   CF = set on error
;   AH = status (01=keyboard controller is in secure mode, 0x86=function not supported)
;
;INT 0x15 Function 2402 - A20 Status
;   Returns:
;
;   CF = clear if success
;   AH = status (01: keyboard controller is in secure mode; 0x86: function not supported)
;   AL = current state (00: disabled, 01: enabled)
;   CX = set to 0xffff is keyboard controller is no ready in 0xc000 read attempts
;   CF = set on error
;
;INT 0x15 Function 2403 - Query A20 support
;Returns:
;
;CF = clear if success
;AH = status (01: keyboard controller is in secure mode; 0x86: function not supported)
;BX = status.
;
;BX contains a bit pattern:
;
;    Bit 0 - supported on keyboard controller
;    Bit 1 - if supported on bit 1 of I/O port 0x92
;    Bits 2:14 - Reserved
;    Bit 15 - 1 if additional data is available.
;
; I/O Port 0x92 infomation:
;
;    Bit 0 - Setting to 1 causes a fast reset
;    Bit 1 - 0: disable A20; 1: enable A20
;    Bit 2 - Manufacturer defined
;    Bit 3 - power on password bytes (CMOS bytes 0x38-0x3f or 0x36-0x3f). 0: accessible, 1: inaccessible
;    Bits 4-5 - Manufacturer defined
;    Bits 6-7 - 00: HDD activity LED off; any other value is "on"
;
EnableA20:
    push bx
    push cx
    ; end prolog

    ; checked this way since this will /always/ work
.a20_check:
    pushf
    push ds
    push es
    push di
    push si
    cli

    xor ax, ax
    mov es, ax

    not ax                      ; ax = 0xFFFF
    mov ds, ax

    mov di, 0x0500              ; scratch location 1
    mov si, 0x0510              ; scratch location 2

    mov al, byte [es:di]
    push ax                     ; save whatever is at 0x0000:0500, physical location 0x0500

    mov al, byte [ds:si]
    push ax                     ; save whatever is at 0xFFFF:0510 [clarification: 0x100500 physical location (0x100500 - 1MB = 0x0500)]

    mov byte [es:di], 0x00      ; zero non-wraped location and write 0xFF to it after (ab)using wrapping
    mov byte [ds:si], 0xFF      ; if the non-wrapped location is 0xFF, then we wraped and A20 is disabled

    cmp byte [es:di], 0xFF

    pop ax
    mov byte [ds:si], al        ; restore original contents of scratch location 2

    pop ax
    mov byte [es:di], al        ; restore original contents of scratch location 1

    mov ax, 0                   ; return 0 if es:di == ds:si (memory wraps)
    je EnableA20.end_check
    mov ax, 1                   ; return 1 if es:di != ds:si (A20 is enabled)
.end_check:
    pop si
    pop di
    pop es
    pop ds
    popf
    sti

    cmp ax, 1
    je EnableA20.endp         ; A20 is already enabled

    mov ax, 0x2403
    int 0x15
    jc EnableA20.do_fallback_a20    ; carry = error...not supported?
    cmp ah, 0
    ja EnableA20.do_fallback_a20   ; non-zero return = error as well

    mov al, bl
    and al, 0000_0010b
    cmp al, 0000_0010b
    je EnableA20.do_fast_a20        ; if fast a20 is supported use it

    jmp EnableA20.do_bios_a20       ; else fall back to enabling via BIOS

.do_fallback_a20:
    ERROR STAGE2_A20_FAILED
.do_bios_a20:
    mov ax, 0x2401
    int 0x15
    jmp EnableA20.a20_check
.do_fast_a20:
    in al, 0x92                 ; read from FAST A20 port
    or al, 2                    ; bit 0 is a fast reset, bit 1 is fast A20
    and al, 0xFE                ; make sure bit 0 is 0
    out 0x92, al                ; enable A20
    jmp EnableA20.a20_check
.endp:
    pop cx
    pop bx

    ret


EnterUnrealMode:
    cli                         ; no interrupts
    push ds                     ; save real mode
    push bx

    lgdt [unreal_gdt_info]

    mov  eax, cr0               ; switch to pmode
    or al,1                     ; set pmode bit
    mov  cr0, eax

    jmp $+2                     ; clear instruction cache

    mov  bx, 0x08               ; select descriptor 1
    mov  ds, bx                 ; 8h = 1000b

    and al,0xFE                 ; back to realmode
    mov  cr0, eax               ; by toggling bit again

    pop bx
    pop ds                      ; get back old segment
    sti
.endp:
    ret

; #############
;
; Strings
;
; #############

%define StrCRLF_NUL 0Dh, 0Ah, 00h

HelloPrompt_cstr:
    db 'Stevia Stage2', StrCRLF_NUL
A20_Enabled_cstr:
    db 'A20 Enabled OK', StrCRLF_NUL
MemoryMap_OK_cstr:
    db 'Memory map OK', StrCRLF_NUL
UnrealMode_OK_cstr:
    db 'Unreal mode OK', StrCRLF_NUL
FileFound_OK_cstr:
    db 'Found SFN entry for bootable binary, first cluster -> ', 00h
IntToHex_table:
    db '0123456789ABCDEF'
NewLine_cstr:
    db StrCRLF_NUL
BootTarget_str:
    db "BOOT_386BIN"

; #############
;
; Locals
;
; #############

partition_offset:
    dw 0x0000



; GDT documentation below:
;
;    Pr: Present bit. This must be 1 for all valid selectors.
;
;    Privl: Privilege, 2 bits. Contains the ring level,
;           0 = highest (kernel), 3 = lowest (user applications).
;
;    S: Descriptor type. This bit should be set for code or data segments
;       and should be cleared for system segments (eg. a Task State Segment)
;
;    Ex: Executable bit. If 1 code in this segment can be executed
;        ie. a code selector. If 0 it is a data selector.
;
;    DC: Direction bit/Conforming bit.
;        Direction bit for data selectors: Tells the direction.
;        0 the segment grows up. 1 the segment grows down, ie. the offset has to be greater than the limit.
;
;        Conforming bit for code selectors:
;            If 1 code in this segment can be executed from an equal or lower privilege level.
;               For example, code in ring 3 can far-jump to conforming code in a ring 2 segment.
;               The privl-bits represent the highest privilege level that is allowed to execute the segment.
;                   For example, code in ring 0 cannot far-jump to a conforming code segment with privl==0x2
;                   while code in ring 2 and 3 can. Note that the privilege level remains the same
;                   ie. a far-jump form ring 3 to a privl==2-segment remains in ring 3 after the jump.
;
;            If 0 code in this segment can only be executed from the ring set in privl.
;
;    RW: Readable bit/Writable bit.
;        Readable bit for code selectors: Whether read access for this segment is allowed. Write access is never allowed for code segments.
;        Writable bit for data selectors: Whether write access for this segment is allowed. Read access is always allowed for data segments.
;
;    Ac: Accessed bit. Just set to 0. The CPU sets this to 1 when the segment is accessed.
;
;    Gr: Granularity bit. If 0 the limit is in 1 B blocks (byte granularity), if 1 the limit is in 4 KiB blocks (page granularity).
;
;    Sz: Size bit. If 0 the selector defines 16 bit protected mode. If 1 it defines 32 bit protected mode.
;        You can have both 16 bit and 32 bit selectors at once.
;
;    AvL: Availible to software bit, the CPU does not use this field and software can read/write to it
;
;    D/B bit: The default operand-size bit is found in code-segment and data-segment descriptors but not in system-segment descriptors. Setting
;              this bit to 1 indicates a 32-bit default operand size, and clearing it indicates a 16-bit default size.
;
;    E bit: Expand down bit: Setting this bit to 1 identifies the data segment as expand-down.
;           In expand-down segments, the segment limit defines the lower segment boundary while the base is the upper boundary
;
; A GDT entry is 8 bytes and is constructed as follows:
; First DWORD
;   0-15	Limit 0:15	First 16 bits in the segment limiter
;   16-31	Base 0:15	First 16 bits in the base address
;
; 2nd DWORD
;
;   0:7	    Base 16:23	Bits 16-23 in the base address
;   8:12	S/Type	    Segment type and attributes, S = bit 12, Type = 8:11, Type is either [1, DC, RW, Ac] <code> or [0, E, RW, Ac] <data>
;   13:14	Privl	    0 = Highest privilege (OS), 3 = Lowest privilege (User applications)
;   15	    Pr	        Set to 1 if segment is present
;   16:19	Limit 16:19	Bits 16-19 in the segment limiter
;   20:22	Attributes	Different attributes, depending on the segment type
;   23	    Gr	        Used together with the limiter, to determine the size of the segment
;   24:31	Base 24:31	The last 24-31 bits in the base address
;
;
;

ALIGN 4, db 0
unreal_gdt_info:
    unreal_gdt_size: dw (unreal_gdt_end - unreal_gdt_start) - 1
    unreal_gdt_ptr:  dd unreal_gdt_start

unreal_gdt_start:
    ; entry 0
    dq 0                    ; first entry is null

    ; entry 1 (4 GiB flat data map)
    ; first dword
    dw 0xFFFF               ; 0:15 limit
    dw 0x0000               ; 0:15 base
    ; second dword
    db 0x00                 ; 16:23 base
    db 1001_0010b            ; bit 0:4 = S/Type, [1, DC, RW, Ac] <code> or [0, E, RW, Ac] <data>
                            ; bit 5:6 = Privl
                            ; bit 7   = Present

    db 1000_1111b            ; bit 0:3 = 16:19 of Limit
                            ; bit 4 = Availible to software bit
                            ; bit 5 = Reserved (?)
                            ; bit 6 = D/B bit, depending on if this is code/data 1 = 32 bit operands or stack size
                            ; bit 7 = Granularity bit. 1 = multiply limit by 4096
    db 0x00                 ; base 24:31
    ; at the end of the day...
    ; base = 0x00000000
    ; limit = 0xFFFFF
    ; Accessed = 0, ignore this field
    ; RW = 1, data is Read/Write
    ; E = 0, Expand up, valid data is from base -> limit, if 1 valid data is from (limit + 1) -> base
    ; C/D = 0, Segment is a data segment
    ; S = 1, Segment is a system segment
    ; Privl = 00b, Ring0 segment
    ; Pr = 1, segment is present
    ; AVL = 0, ignore this field
    ; D/B bit = 0, 16bit code/stack
    ; Gr = 1, multiply limit by 4096
unreal_gdt_end:

ALIGN 4, db 0
gdt32_info:
    gdt32_size: dw (gdt32_end - gdt32_start) - 1
    gdt32_ptr:  dd gdt32_start

; check above for detailed information
gdt32_start:
    dq 0
    .gdt32_code:
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_1000b
        db 1100_1111b
        db 0x00
    .gdt32_data:
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_0010b
        db 1100_1111b
        db 0x00
gdt32_end:

%assign bytes_remaining ((MAX_STAGE2_BYTES - 4) - ($ - $$))
%warning STAGE2 has bytes_remaining bytes remaining for code (MAX: MAX_STAGE2_BYTES)

; this is here to make the stage2 bigger than 1 sector for testing
times ((MAX_STAGE2_BYTES - 4) - ($ - $$)) db 0x00
STAGE2_SIG: dd 0xDEADBEEF