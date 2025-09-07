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

%ifndef __INC_KMEMCPY5_FUNC
%include 'cdecl16.inc'

; uint8_t* kmemset(word dest_segment, word dest, word src_segment, word src, byte len);
; not overlap safe, only for
ALIGN 4, db 0x90
kmemcpy5:
    __CDECL16_PROC_ENTRY
    push ds
    push es
.setup_segments:
    mov ax, [bp + 4]
    mov ds, ax              ; destination segment

    mov ax, [ bp + 8]
    mov es, ax              ; source segment
.func:
    mov cx, [bp + 12]        ; len
    mov si, [bp + 10]        ; src
    mov di, [bp + 6]        ; dest
    
    cld                     ; ensure we are incrementing
    rep movsb               ; move ds:si -> es:di
    mov ax, di              ; return pointer to dest
.restore_segments:
    pop es
    pop ds
.endf:
    __CDECL16_PROC_EXIT
    ret

%define __INC_KMEMCPY5_FUNC
%endif