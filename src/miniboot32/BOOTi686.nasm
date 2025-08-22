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

[BITS 32]
[ORG 0x100000]
[CPU KATMAI]
jmp short (init32 - $$)         ; PI jump
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