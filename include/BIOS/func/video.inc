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

; Sets output to 80x25 16 color text mode via BIOS call
; also clears screen
; void SetTextMode(void)
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