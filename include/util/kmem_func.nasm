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

%ifndef __INC_KMEM_FUNC
%include 'cdecl16.inc'

; uint8_t* kmemset_byte(void* dst, uint8_t val, uint16_t len);
ALIGN 4, db 0x90
kmemset:
    __CDECL16_ENTRY
 .func:
    mov     cx, [bp + 8]   ; size_t len
    mov     al, [bp + 6]   ; uint8_t val
    mov     di, [bp + 4]   ; void * dst

    cld
    rep     stosb
    mov     ax, di         ; return pointer to dest
.endp:
    __CDECL16_EXIT
    ret

; uint8_t* kmemset(uint8_t* dest, uint8_t* src, uint8_t len);
; not overlap safe
ALIGN 4, db 0x90
kmemcpy:
    __CDECL16_ENTRY
.func:
    mov cx, [bp + 8]        ; len
    mov si, [bp + 6]        ; src
    mov di, [bp + 4]        ; dest
    
    cld                     ; ensure we are incrementing
    rep movsb
    mov ax, di              ; return pointer to dest
.endf:
    __CDECL16_EXIT
    ret

%endif
%define __INC_KMEM_FUNC