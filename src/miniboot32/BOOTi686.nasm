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

[BITS 32]
[ORG 0x100000]
[CPU KATMAI]
jmp short init32
nop

;;;
; Errors
;   0 = unused
;   E = General Error
;   S = magic signature not found at end of file
;   O = OK
;;;

%define MAX_BYTES (1024 * 8)
%define VGA_BUF 0xb8000

; VGA memory is row-major => offset = (row * MAX_Colums) + colum
; macro array is counted from 0,0 = 1,1 => 79,24 is the last usable space in a page
%define VGA_OFFSET(x,y) (((y*VGA_MAX_X) + x)*2)
;subtract 1 for max array values
%define VGA_MAX_Y 25
%define VGA_MAX_X 80

ALIGN 16, db 0
init32:
    mov ax, 0x10        ; 0x10 selector segment
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax          ; load data registers with 2nd GDT selector
    mov esp, stack_top
    mov ebp, esp

    mov eax, dword [BOOT_SIG]
    cmp eax, 0xA0B0C0D0
    jz .signature_present

    mov dl, "S"
    jmp .result
.signature_present:
    mov dl, "O"
.result:
    mov eax, VGA_BUF
    add eax, VGA_OFFSET(73, 24)
    
    mov dword [eax], 0x1f451f52
    add eax, 0x4
    mov dword [eax], 0x1f3e1f53
    add eax, 0x4
    ; RES> in white-on-blue

    mov ecx, 0x1f201f20
    and cl, dl                          ; 0x1f201f(dl)
    ror ecx, 16                         ; 0x1f(dl)1f20 = ' (dl)'
    mov dword [eax], ecx                ; should be a space and contents of dl
.endp:
    hlt
    jmp .endp - $$

; Strings
ALIGN 16, db 0
%define StrCRLF_NUL 0Dh, 0Ah, 00h
version_cstr:
    db 'CBoot v0.0.3 ', 'NASM - ', __NASM_VER__, StrCRLF_NUL
datetime_cstr:
    db 'Assembled - ', __DATE__, ' ', __TIME__, StrCRLF_NUL

ALIGN 16, db 0
stack_bottom:
    times (512 * 8) db 0x00       ; 4KiB stack
stack_top:

%assign bytes_remaining ((MAX_BYTES - 4) - ($ - $$))
%warning boot32 has bytes_remaining bytes remaining for code (MAX: MMAX_BYTES)

times ((MAX_BYTES - 4) - ($ - $$)) db 0xFE
BOOT_SIG: dd 0xA0B0C0D0