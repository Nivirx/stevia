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

%ifndef __INC_KMEM_FUNC
%include 'cdecl16.inc'

; void* kmemset_byte(void* dst, uint8_t val, uint16_t len);
ALIGN 4, db 0x90
kmemset:
    __CDECL16_PROC_ENTRY
 .func:
    mov     cx, [bp + 8]        ; uint16_t len
    mov     al, byte [bp + 6]   ; uint8_t val
    mov     di, [bp + 4]        ; void * dst

    cld
    rep     stosb
    mov     ax, di         ; return pointer to dest + len (last elem of dest)
.endp:
    __CDECL16_PROC_EXIT
    ret

; uint8_t* kmemset(uint16_t* dest, uint16_t* src, uint16_t len);
; not overlap safe
ALIGN 4, db 0x90
kmemcpy:
    __CDECL16_PROC_ENTRY
.func:
    mov cx, [bp + 8]        ; len
    mov si, [bp + 6]        ; src
    mov di, [bp + 4]        ; dest
    
    cld                     ; ensure we are incrementing
    rep movsb
    mov ax, di              ; return pointer to dest
.endf:
    __CDECL16_PROC_EXIT
    ret

%endif
%define __INC_KMEM_FUNC