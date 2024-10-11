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
%ifnmacro __CDECL16_ENTRY
%macro __CDECL16_ENTRY 0
    push bp
    mov bp, sp
    sub sp, 0x10

    push si
    push di
    push bx
%endmacro
%endif

%ifnmacro __CDECL16_EXIT
%macro __CDECL16_EXIT 0
    pop bx
    pop di
    pop si

    mov sp, bp
    pop bp
%endmacro
%endif

%ifnmacro __CDECL16_CALLER_SAVE
%macro __CDECL16_CALLER_SAVE 0
    push ax
    push cx
    push dx
%endmacro
%endif

%ifnmacro __CDECL16_CALLER_RESTORE
%macro __CDECL16_CALLER_RESTORE 0
    pop dx
    pop cx
    pop ax
%endmacro
%endif