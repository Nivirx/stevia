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

%ifndef __INC_KMEMCPY5_FUNC
%include 'cdecl16.inc'

; uint8_t* kmemset(word dest_segment, word dest, word src_segment, word src, byte len);
; not overlap safe, only for
ALIGN 4, db 0x90
kmemcpy5:
    __CDECL16_ENTRY
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
    __CDECL16_EXIT
    ret

%define __INC_KMEMCPY5_FUNC
%endif