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

%ifndef __INC_KMEMSET4_FUNC
%include 'cdecl16.inc'

; word kmemset_byte(word segment, word dst, byte val, word len);
ALIGN 4, db 0x90
kmemset4:
    __CDECL16_ENTRY
.setup_segment:
    push es
    mov ax, [bp + 4]
    mov es, ax
 .func:
    mov     cx, [bp + 10]       ; size_t len
    mov     al, [bp + 8]        ; uint8_t val
    mov     di, [bp + 6]        ; word dst

    cld
    rep     stosb               ; move al -> es:di
    mov     ax, di              ; return pointer to dest
.restore_segments:
    pop es
.endf:
    __CDECL16_EXIT
    ret

%endif
%define __INC_KMEMSET4_FUNC