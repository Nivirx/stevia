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
    __CDECL16_PROC_ENTRY
    pushf
.func:
    xor ah, ah                  ; Set Video mode BIOS function
    mov al, 0x03                ; 16 color 80x25 Text mode
    int 0x10                    ; Call video interrupt
                                

    mov ah, 0x05                ; Select active display page BIOS function
    xor al, al                  ; page 0
    int 0x10                    ; call video interrupt
.endp:
    popf
    __CDECL16_PROC_EXIT
    ret

; disables blinking text mode cursor via crtc pokes
; 0x3D4/0x3D5 for color, mono at 0x3B4/0x3B5
ALIGN 4, db 0x90
disable_cursor_crtc:
    __CDECL16_PROC_ENTRY
.func:
    mov dx, 0x3D4
	mov al, 0xA	    ; low cursor shape register
	out dx, al
 
	inc dx
	mov al, 0x20	; bits 6-7 unused, bit 5 disables the cursor, bits 0-4 control the cursor shape
	out dx, al
.endp:
    __CDECL16_PROC_EXIT
    ret

; disables blinking text mode cursor via BIOS int 10h, ah = 01
ALIGN 4, db 0x90
disable_cursor_bios:
    __CDECL16_PROC_ENTRY
.func:
    push bx
    mov  ah, 03h      ; read cursor pos & shape
    xor  bh, bh
    int  10h          ; CH=top scanline, CL=bottom

    or   ch, 20h      ; set bit 5 = disable
    mov  ah, 01h      ; set leaf 01h
    int  10h
    pop  bx
.endp:
    __CDECL16_PROC_EXIT
    ret

; Prints a C-Style string (null terminated) using BIOS vga teletype call
; void PrintString(char* buf)
ALIGN 4, db 0x90
PrintString:
    __CDECL16_PROC_ENTRY
    mov di, [bp + 4]    ; first arg is char[] 
.str_len:
    xor cx, cx         ; ECX = 0
    not cx             ; ECX = -1 == 0xFFFF
    xor ax, ax         ; search for al = 0x0

    cld
    repne scasb        ; deincrement cx while searching for al

    not cx             ; the inverse of a neg number = abs(x) - 1
    dec cx             ; CX contains the length of the string - nul byte at end
.print:
    mov si, [bp + 4]            ; source string
.print_L0:
    movzx ax, byte [si]
    %ifdef __STEVIA_DEV_DEBUG
    out 0xe9, byte al           ; bochs magic debug port
    %endif
    push ax
    call PrintCharacter
    add sp, 0x2

    inc si
    dec cx

    jcxz PrintString.endp
    jmp PrintString.print_L0    ; Fetch next character from string
.endp:
    __CDECL16_PROC_EXIT
    ret                         ; Return from procedure

; Prints a single character
; void PrintCharacter(char c);
ALIGN 4, db 0x90
PrintCharacter:
    __CDECL16_PROC_ENTRY
.func:
    mov al, byte [bp + 4]       ; AL = character c
    mov ah, 0x0E                ; INT 0x10, AH=0x0E call
    mov bx, 0x0007              ; BH = page no. BL =Text attribute
    int 0x10                    ; call video interrupt
                                ; TODO: check for carry and clear carry before call
.endp:
    __CDECL16_PROC_EXIT
    ret

; TODO: fix the prolog, epilog and stack usage to confirm with cdecl16
; prints the hex representation of of val
; void PrintDWORD(uint32_t val);
ALIGN 4, db 0x90
PrintDWORD:
    __CDECL16_PROC_ENTRY
.func: 
    mov si, IntToHex_table
    mov ebx, 16     ; base-16

    mov eax, dword [bp + 4]     ;val

    xor edx, edx
    xor cx, cx
.next_digit:
    div ebx            ; dividend in edx:eax -> quotient in eax, remainder in edx
    push dx            ; save remainder
    inc cx

    xor dx, dx
    test eax, eax
    jnz PrintDWORD.next_digit

.zero_pad:
    cmp cx, 0x0008
    je PrintDWORD.print_stack
    xor ax, ax
    push ax
    inc cx
    jmp PrintDWORD.zero_pad

.print_stack:
    pop bx
    dec cx
    push cx

    movzx ax, byte [bx+si+0]    ; bx = index into Hex lookup table
    push ax
    call PrintCharacter
    add sp, 0x2

    pop cx

    jcxz PrintDWORD.endp
    jmp PrintDWORD.print_stack

.endp:
    __CDECL16_PROC_EXIT
    ret
%endif
%define __INC_VIDEO