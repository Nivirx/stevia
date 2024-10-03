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

; uint8_t* kmemset(void* dest, uint8_t val, size_t len);
kmemset:
    push    di             ; function uses di, so save it.
 
    mov     cx, [bp - 2]   ; size_t len
    mov     al, [bp - 4]   ; uint8_t val
    mov     di, [bp - 6]   ; void * ptr

    cld
    rep     stosb 
.endf:
    mov     ax, [bp - 6]  ; return pointer to dest
    pop     di            ; restore di
    ret

; uint8_t* kmemset(uint8_t* dest, uint8_t* src, size_t len);
; not overlap safe
kmemcpy:
    push di
    push si                 ; di, si are callee save

    mov cx, [bp - 2]        ; length
    mov si, [bp - 4]        ; source
    mov di, [bp - 6]        ; dest
    
    cld                     ; ensure we are incrementing
    rep movsb

.endf:
    mov ax, [bp - 6]        ; return pointer to dest
    pop si
    pop di
    ret