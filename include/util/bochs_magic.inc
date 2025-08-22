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

%ifndef __INC_BOCHS_DEBUG_MAGIC

; for things like the error printer, opperate differently when we are targeting a dev (bochs) build
%define __STEVIA_DEV_DEBUG
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

%define __INC_BOCHS_DEBUG_MAGIC
%endif