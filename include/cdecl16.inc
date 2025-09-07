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

; __CDECL16_PROC_ARGS nargs
;   Creates %$arg1 .. %$argN as [bp+4], [bp+6], ...
;   for use inside of function bodies
%ifnmacro __CDECL16_PROC_ARGS
%macro __CDECL16_PROC_ARGS 1
    %push __CDECL16_PROC_ARGS
    %assign %$__i 1
    %rep %1
        %xdefine %$arg%$__i  [bp + 4 + 2*(%$__i-1)]
        %assign  %$__i %$__i + 1
    %endrep
    %pop
%endmacro
%endif

; __CDECL16_CALL_ARGS a,b,c  ==> push c / push b / push a
;   handles pushing values to the stack in the correct order
%ifnmacro __CDECL16_CALL_ARGS
%macro __CDECL16_CALL_ARGS 1-*
    %rep %0
        %rotate -1      ; move the last argument into %1
        push %1         ; push last, then next-to-last, ...
    %endrep
%endmacro
%endif

; __CDECL16_CALL func, nargs   ; adds sp by nargs*2 after call
;   adds up word pushes to restore stack frame
;   WARNING: if you push a dword it will only count 2, use __CDECL16_CALL_ARGS_SIZED
%ifnmacro __CDECL16_CALL
%macro __CDECL16_CALL 2
    call %1
    add  sp, %2*2
%endmacro
%endif

; __CDECL16_CALL_ARGS_SIZED func, size1[, size2 ...]  ; sizes in BYTES
;   if you *need* to pass dword sized args in 16-bit mode, use this to properly
;   count the stack frame to restore later!
%ifnmacro __CDECL16_CALL_ARGS_SIZED
%macro __CDECL16_CALL_ARGS_SIZED 2-*
    %push __CDECL16_CALL_ARGS_SIZED
    call %1
    %assign %$__sum 0
    %assign %$__i 2
    %rep %0-1
        %assign %$__sum %$__sum + %[%$__i]   ; sizes may be expressions
        %assign %$__i %$__i + 1
    %endrep
    %if %$__sum & 1
        %error "__CDECL16_CALL_ARGS_SIZED: total size is odd; pushes must sum to whole bytes actually pushed (usually even)."
    %endif
    add sp, %$__sum
    %pop
%endmacro
%endif


; __CDECL16_INVOKE func, arg1[, arg2 ...]
;   pushes args right-to-left, calls, then add sp by (#args)*2
;   !!! warning for word sized args only !!!
%ifnmacro __CDECL16_INVOKE
%macro __CDECL16_INVOKE 1-*
    %push __CDECL16_INVOKE
    %define %$fn %1
    %assign %$argc %0-1
    %rotate -1              ; drop function name; %1.. are args
    %rep %$argc
        %rotate -1
        push %1
    %endrep
    call %$fn
    %if %$argc
        add  sp, %$argc*2
    %endif
    %pop
%endmacro
%endif

%ifnmacro __CDECL16_CLOB
; Save registers in the order listed. Supports "flags" as a pseudo-reg.
%macro __CDECL16_CLOB 1-*
    %rep %0
        %ifidni %1, flags
            pushf
        %else
            push %1
        %endif
        %rotate 1
    %endrep
%endmacro
%endif

%ifnmacro __CDECL16_UNCLOB
; Restore in reverse order of CLOB. Supports "flags".
%macro __CDECL16_UNCLOB 1-*
    %rep %0
        %rotate -1
        %ifidni %1, flags
            popf
        %else
            pop %1
        %endif
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