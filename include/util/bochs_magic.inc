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

%ifndef __INC_BOCHS_DEBUG_MAGIC

;
; port_e9_hack: enabled=1 needs to be set in bochsrc.txt/bxrc
;
%ifnmacro __BOCHS_PRINTC
    %macro __BOCHS_PRINTC 1
        mov al, %1
        out 0xe9, byte al
    %endmacro
%endif

;
; port_e9_hack: enabled=1 needs to be set in bochsrc.txt/bxrc
;
%ifnmacro __BOCHS_BREAK
    %macro __BOCHS_BREAK 0
        push ax
        mov ax, 0x8A00
        out 0x8A00, word ax
        pop ax
    %endmacro
%endif

;
; magic_break: enabled=1 needs to be set in bochsrc.txt/bxrc
;
%ifnmacro __BOCHS_MAGIC_DEBUG
    %macro __BOCHS_MAGIC_DEBUG 0
        xchg bx, bx
    %endmacro
%endif

%ifnmacro PRINT_FUNC_LABEL
    %macro PRINT_FUNC_LABEL 1
        %define label %1
        %assign addr label
        %ifnctx __OUTPUT__
            __OUTPUT__ "Entry point for '%1' = %d", addr
        %endif
    %endmacro
%endif

%define __INC_BOCHS_DEBUG_MAGIC
%endif