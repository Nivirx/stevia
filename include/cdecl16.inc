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

%ifnmacro __CDECL16_ENTRY
%macro __CDECL16_ENTRY 0-1
    push bp
    mov bp, sp

    push si
    push di
    push bx

    %if %0 = 1
    sub  sp, %1        ; reserve locals only when needed
    %endif
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

%ifnmacro __FASTCALL16_ENTRY
%macro __FASTCALL16_ENTRY 0
    push bp
    mov bp, sp
%endmacro
%endif

%ifnmacro __FASTCALL16_EXIT
%macro __FASTCALL16_EXIT 0
    mov sp, bp
    pop bp
%endmacro
%endif

%ifnmacro __TINY_DS_ES
%macro __TINY_DS_ES 0
    mov ax, __STAGE2_SEGMENT
    mov ds, ax
    mov es, ax
%endmacro
%endif

%ifnmacro __REFLAT_DS_ES
%macro __REFLAT_ES 0
    cli                                   ; no interrupts   
    lgdt [((__STAGE2_SEGMENT << 4) + unreal_gdt_info)]                 ; load unreal gdt

    mov eax, cr0                          
    or eax, 1                             ; set pmode bit
    mov cr0, eax                          ; switch to pmode
    jmp short $+2                         ; i-cache flush

    mov ax, 0x10                          ; select descriptor 2
    mov ds, ax                            ; 10h = 0001_0000b
    mov es, ax                            ; es to big data

    mov eax, cr0
    and eax, ~1                           ; toggle bit 1 of cr0
    mov cr0, eax                          ; back to realmode
    jmp short $+2                         ; i-cache flush
    sti

%endmacro
%endif