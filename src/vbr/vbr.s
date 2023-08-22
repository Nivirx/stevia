; Copyright (c) 2023 Elaina Claus
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


[BITS 16]
[ORG 0x7C00]
[CPU KATMAI]
jmp short init
nop

bpb_start:
; fill BPB area with 0x00 since we skip writing this part to disk
; but we need it for the 'jmp short entry; nop' above
times 33 db 0x00

ebpb_start:
; fill BPB area with 0x00 since we skip writing this part to disk
; but we need it for the 'jmp short entry; nop' above
times 54 db 0x00

%include "entry.inc"
init:
    cli                         ; We do not want to be interrupted

    xor ax, ax                  ; 0 AX
    mov ds, ax                  ; Set segment registers to 0
    mov es, ax                  ; *

    mov ss, ax                  ; Set Stack Segment to 0
    mov sp, STACK_START         ; Setup stack
    mov bp, sp                  ; base ptr = stack ptr

    mov bx, VBR_ENTRY           ; move BP to the new start of the initial boot sector

    jmp 0:main                  ; fix up cs:ip just in case and jump to relocated code

%include "config.inc"
%include "errors.inc"
%include "memory.inc"

%include "partition_table.inc"

%include "fat32/bpb.inc"



main:
    sti                         ; all done with inital setup and relocation, reenable interupts
    mov [bsDriveNumber], dl     ; BIOS passes drive number in DL
    mov [partition_offset], si  ; save passed partition entry offset

.check_FAT_size:
    cmp dword [bsSectorHuge], 0      ; SectorsHuge will not be set if FAT12/16
    ja main.load_stage2

    ERROR VBR_ERROR_WRONG_FAT_SIZE

; read sectors 1-63 to stage2 entry point
.load_stage2:
    push bp
    mov bp, sp

    ;uint8_t read_disk_raw(size_t count, uint16_t buf_segment, uint16_t buf_offset, 
    ;                      uint16_t lower_lower_lba, uint16_t upper_lower_lba)

    xor ax, ax
    push ax         ; upper_lower_lba = 0

    mov ax , 1
    push ax         ; lower_lower_lba = 1

    xor ax, ax
    push ax          ; offset = 0

    ; 07E0:0
    mov ax, STAGE2_ENTRY
    shr ax, 4
    push ax          ; segment = 7E0

    mov ax, STAGE2_SECTOR_COUNT
    push ax

    call read_disk_raw
    leave

.check_sig:
    mov ax, 0x7E0
    mov fs, ax
    cmp dword [fs:0x7FFC], 0xDEADBEEF
    je main.sig_ok

    ERROR VBR_ERROR_NO_SIGNATURE          ; no signature present in stage2
    
.sig_ok:
    push bp
    mov bp, sp
    mov ax, fat32_bpb_SIZE          ; size in byte
    push ax
    mov ax, bpb_start               ; start of bpb
    push ax
    mov ax, fat32_bpb               ; defined in memory.inc, destination
    push ax
    call kmemcpy                    ; copy bpb to memory
    leave

    push bp
    mov bp, sp
    mov ax, fat32_ebpb_SIZE          ; 72 bytes of data
    push ax
    mov ax, ebpb_start               ; start of ebpb
    push ax
    mov ax, fat32_ebpb               ; defined in memory.inc, destination
    push ax
    call kmemcpy                     ; copy ebpb to memory
    leave

    mov si, [partition_offset]
    mov dl, [bsDriveNumber]
    jmp 0:0x7E00

stop:
    hlt
    jmp short stop

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
; bp-10 = count
; bp-12 = ret ptr
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
    mov word [lba_packet + LBAPkt_t.xfer_size], STAGE2_SECTOR_COUNT

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
    mov dl, byte [bsDriveNumber]
    int 0x13
    jnc read_disk_raw.endf

    ERROR VBR_ERROR_DISK_READ_ERR
.endf:
    pop si
    ret
; Data

; #############
;
; Locals
;
; #############

; offset from the begining of sector 0 to the active partition.
partition_offset:
    dw 0x0000

%assign bytes_remaining (420 - ($ - $$))
%warning VBR has bytes_remaining bytes remaining for code (MAX: 420 bytes)

times (510 - ($ - $$)) nop     ; Fill the rest of sector with nop

BootSig:
    dw 0xAA55                    ; Add boot signature at the end of bootloader