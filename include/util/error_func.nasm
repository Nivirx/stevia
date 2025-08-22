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

%ifndef __INC_ERROR_FUNC

%macro ERROR 1
    %ifdef __STEVIA_DEV_DEBUG
    __BOCHS_MAGIC_DEBUG
    %endif
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