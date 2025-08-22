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

%ifndef __INC_VIDEO
; Sets output to 80x25 16 color text mode via BIOS call
; also clears screen
; void SetTextMode(void)
ALIGN 4, db 0x90
SetTextMode:
.prolog:
    __CDECL16_ENTRY
    pushf
.func:
    xor ah, ah                  ; Set Video mode BIOS function
    mov al, 0x02                ; 16 color 80x25 Text mode
    int 0x10                    ; Call video interrupt

    mov ah, 0x05                ; Select active display page BIOS function
    xor al, al                  ; page 0
    int 0x10                    ; call video interrupt
.endp:
    popf
    __CDECL16_EXIT
    ret

; disables blinking text mode cursor
ALIGN 4, db 0x90
disable_cursor:
    __CDECL16_ENTRY
.func:
    mov dx, 0x3D4
	mov al, 0xA	    ; low cursor shape register
	out dx, al
 
	inc dx
	mov al, 0x20	; bits 6-7 unused, bit 5 disables the cursor, bits 0-4 control the cursor shape
	out dx, al
.endp:
    __CDECL16_EXIT
    ret
    
%endif
%define __INC_VIDEO