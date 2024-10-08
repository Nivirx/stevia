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

%ifndef __INC_A20ENABLE
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
ALIGN 4, db 0x90
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
    
%endif
%define __INC_A20ENABLE