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

%ifnmacro __CDECL16_PROC_ENTRY
%macro __CDECL16_PROC_ENTRY 0-1
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

%ifnmacro __CDECL16_PROC_EXIT
%macro __CDECL16_PROC_EXIT 0
    pop bx
    pop di
    pop si

    mov sp, bp
    pop bp
%endmacro
%endif

; __CDECL16_ARGS nargs
;   Creates %$arg1 .. %$argN as [bp+4], [bp+6], ...
;   for use inside of function bodies
%ifnmacro __CDECL16_ARGS
%macro __CDECL16_ARGS 1
    %assign %$__i 1
    %rep %1
        %xdefine %$arg%$__i  [bp + 4 + 2*(%$__i-1)]
        %assign  %$__i %$__i + 1
    %endrep
%endmacro
%endif

; __CDECL16_PUSH a,b,c  ==> push c / push b / push a
;   handles pushing values to the stack in the correct order
%ifnmacro __CDECL16_PUSH
%macro __CDECL16_PUSH 1-*
    %assign %$__i %0
    %rep %0
        push %[%$__i]
        %assign %$__i %$__i - 1
    %endrep
%endmacro
%endif

; __CDECL16_CALL func, nargs   ; adds sp by nargs*2 after call
;   adds up word pushes to restore stack frame
;   WARNING: if you push a dword it will only count 2, use __CDECL16_CALL_SIZED
%ifnmacro __CDECL16_CALL
%macro __CDECL16_CALL 2
    call %1
    add  sp, %2*2
%endmacro
%endif

; __CDECL16_CALL_SIZED func, size1[, size2 ...]  ; sizes in BYTES
;   if you *need* to pass dword sized args in 16-bit mode, use this to properly
;   count the stack frame to restore later!
%ifnmacro __CDECL16_CALL_SIZED
%macro __CDECL16_CALL_SIZED 2-*
    call %1
    %assign %$__sum 0
    %assign %$__i 2
    %rep %0-1
        %assign %$__sum %$__sum + %[%$__i]
        %assign %$__i %$__i + 1
    %endrep
    add sp, %$__sum
%endmacro
%endif

; __CDECL16_INVOKE func, arg1[, arg2 ...]
;   pushes args right-to-left, calls, then add sp by (#args)*2
;   !!! warning for word sized args only !!!
%ifnmacro __CDECL16_INVOKE
%macro __CDECL16_INVOKE 1-*
    %assign %$__argc %0-1
    %if %$__argc > 0
        ; push args right-to-left
        %assign %$__i %0
        %rep %$__argc
            push %[%$__i]
            %assign %$__i %$__i - 1
        %endrep
        call %1
        add  sp, %$__argc*2
    %else
        call %1
    %endif
%endmacro
%endif

%ifnmacro CLOB
; e.g CLOB ax dx ... inside func entry, then UNCLOB ax dx at all exits
; ordering is already taken care of 
; i.e if you CLOB ax cx dx then UNCLOB ax cx dx, the unclob is reversed
%macro __CDECL16_CLOB 1-*
    %assign %$i 1
    %rep %0
        push %[%$i]
        %assign %$i %$i+1
    %endrep
%endmacro
%endif

%ifnmacro __CDECL16_UNCLOB
%macro UNCLOB 1-*
    %assign %$i %0
    %rep %0
        pop %[%$i]
        %assign %$i %$i-1
    %endrep
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