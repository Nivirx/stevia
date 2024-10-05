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

;
;INT 0x15 Function 2400 - Disable A20
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
EnableA20:
    __CDECL16_ENTRY
    push ds
    push es
.a20_check:
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
    pop es
    pop ds
    __CDECL16_EXIT
    ret

; See memory.inc for a brief description of E820 mmap function
GetMemoryMap:
    __CDECL16_ENTRY
    push es         ; save segment register
.func:
    mov dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries], 0

    mov eax, 0xE820                         ; select 0xE820 function
    xor ebx, ebx                            ; Continuation value, 0 for the first call

    mov dx, (BIOSMemoryMap >> 4)
    mov es, dx
    xor di, di                              ; (BIOSMemoryMap >> 4):0 makes di an index into BIOSMemoryMap

    mov ecx, AddressRangeDescStruct_t_size
    mov edx, 0x534D4150                     ; 'SMAP' magic
.loop_L1:
    int 0x15
    jc GetMemoryMap.error
    cmp eax, 0x534D4150
    jne GetMemoryMap.no_smap_returned
.no_error:
    inc dword [SteviaInfo + SteviaInfoStruct_t.MemoryMapEntries]

    cmp ecx, 20                             ; TODO: maybe this could be handled better than just panicing
    jb GetMemoryMap.nonstandard_e820        ; non-standard entry found 

    cmp ebx, 0
    je GetMemoryMap.endp                    ; 0 in ebx means we have reached the end of memory ranges

    add di, AddressRangeDescStruct_t_size   ; increment di to next descriptor
    mov edx, eax                            ; 'SMAP' to edx
    mov eax, 0xE820                         ; select E820 function
    jmp GetMemoryMap.loop_L1
.error:
    ERROR STAGE2_MM_E820_MISC_ERR
.nonstandard_e820:
    ERROR STAGE2_MM_E820_NONSTANDARD
.no_smap_returned:
    ERROR STAGE2_MM_E820_NO_SMAP
.endp:
    pop es
    __CDECL16_EXIT
    ret

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
; uint8_t read_disk_raw(uint16_t buf_segment, uint16_t buf_offset, uint32_t lba)
; TODO: this needs validation
read_disk_raw:
    __CDECL16_ENTRY
.func:  
    mov ax, 0x10
    push ax             ; len = 16 bytes
    xor ax, ax
    push ax             ; val = 0
    mov ax, lba_packet
    push ax             ; dest = lba_packet address

    call kmemset        ; uint8_t* kmemset(void* dest, uint8_t val, size_t len);
    add sp, 0x6

    mov byte [lba_packet + LBAPkt_t.size], 0x10
    mov word [lba_packet + LBAPkt_t.xfer_size], 0x0001

    mov dword eax, [bp + 8]
    mov dword [lba_packet + LBAPkt_t.lower_lba], eax

    mov ax, [bp + 6]
    mov word [lba_packet + LBAPkt_t.offset], ax

    mov ax, [bp + 4]
    mov word [lba_packet + LBAPkt_t.segment], ax

    mov si, lba_packet
    mov ah, 0x42
    mov dl, byte [fat32_ebpb + FAT32_ebpb_t.drive_number_8]
    int 0x13
    jnc read_disk_raw.endp
    ERROR STAGE2_MBR_DISK_READ_ERROR
.endp:
    __CDECL16_EXIT
    ret

; Sets output to 80x25 16 color text mode via BIOS call
; also clears screen
; void SetTextMode(void)
SetTextMode:
.prolog:
    __CDECL16_ENTRY
    pushf
.func:
    xor ah, ah                  ; Set Video mode BIOS function
    mov al, 0x02                ; 16 color 80x25 Text mode
    int 0x10                    ; Call video interrupt

    mov ah, 0x05                ; Select active display page BIOS function
    xor al, al                  ; page 0
    int 0x10                    ; call video interrupt
.endp:
    popf
    __CDECL16_EXIT
    ret

; disables blinking text mode cursor
disable_cursor:
    __CDECL16_ENTRY
.func:
    mov dx, 0x3D4
	mov al, 0xA	    ; low cursor shape register
	out dx, al
 
	inc dx
	mov al, 0x20	; bits 6-7 unused, bit 5 disables the cursor, bits 0-4 control the cursor shape
	out dx, al
.endp:
    __CDECL16_EXIT
    ret