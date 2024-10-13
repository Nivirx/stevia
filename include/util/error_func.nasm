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

%ifndef __INC_ERROR_FUNC

%macro ERROR 1
    mov al, %1           ; al = 1 byte error code mapped to ascii values
    jmp error
%endmacro

; pass error as ascii character in al, errors a-zA-Z or 0-9
ALIGN 4, db 0x90
error:
    cmp al, STEVIA_DEBUG_OK
    jge short .debug            ; the 'letter >= W' (W, X, Y, Z) are used as special debug codes
    mov ah, 0x4F                ; color 0x4F is white text/red background
    jmp short .print
.debug:
    mov ah, 0x5F                ; debug case is white text/purple background
.print:
    mov dx, 0xB800
    mov gs, dx                  
    mov word [gs:0x0000], ax    ; 0xB8000 = video memory
.halt:
    cli
    hlt
    jmp short .halt

%endif
%define __INC_ERROR_FUNC