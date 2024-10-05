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

%macro ERROR 1
    xor ax, ax
    mov al, %1
    jmp error
%endmacro

; pass error as ascii character in al, errors a-zA-Z or 0-9
error:
    ; fs = 0xb800 => fs:0x0000 = 0xb8000
    mov dx, 0xB800
    mov fs, dx

    mov dx, STEVIA_DEBUG_OK
    cmp ax, dx
    jge error.debug_err
    mov dh, 0x4F                ; color 0x4F is white text/red background
    jmp error.print
.debug_err:
    mov dh, 0x5F                ; debug case is white text/purple background

    ; the characters are two bytes in the order of 0xb8000: byte c, byte attribute
    ; since x86 is le, we store the attribute in the MSB of dx
.print:
    mov dl, al
    mov word [fs:0x0000], dx
    jmp error.stop

.stop:
    hlt
    jmp short error.stop