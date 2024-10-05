; Copyright (c) 2023 Elaina Claus
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

[BITS 16]
[ORG 0X7E00]
[CPU KATMAI]
jmp short init
nop

; boot drive in dl
; active partition offset in si
%include "cdecl16.inc"
%include "entry.inc"
init:
    cli                         ; We do not want to be interrupted

    xor ax, ax                  ; 0 AX
    mov ds, ax                  ; Set segment registers to 0
    mov es, ax                  ; *
    mov fs, ax                  ; *
    mov gs, ax                  ; *

    mov ss, ax                  ; Set Stack Segment to 0
    mov sp, EARLY_STACK_START         ; Set Stack Pointer

    add sp, 0x4
    mov ax, 0xDEAD
    push word ax
    mov ax, 0xBEEF
    push word ax             ; mark top of stack for debuging

    mov bp, sp
    sti

    jmp 0:main

%include "config.inc"
%include "error_codes.inc"
%include "memory.inc"
%include "kmem_func.inc"
%include "partition_table.inc"

%include "fat32/bpb.inc"
%include "fat32/fat32_structures.inc"

main:
    mov byte [fat32_ebpb + FAT32_ebpb_t.drive_number_8], dl
    mov word [partition_offset], si

    mov eax, dword [STAGE2_SIG]
    cmp eax, 0xDEADBEEF
    je main.signature_present

    ERROR STAGE2_SIGNATURE_MISSING

.signature_present:
    call SetTextMode
    call disable_cursor

    lea ax, [HelloPrompt_cstr]
    push ax
    call PrintString
    add sp, 0x2

    ; enable A20 gate
    call EnableA20
    
    lea ax, [A20_Enabled_cstr]
    push ax
    call PrintString
    add sp, 0x2

    ; enter unreal mode
    call EnterUnrealMode

    lea ax, [UnrealMode_OK_cstr]
    push ax
    call PrintString
    add sp, 0x2

    ; get system memory map
    call GetMemoryMap

    lea ax, [MemoryMap_OK_cstr]
    push ax
    call PrintString
    add sp, 0x2

    ; FAT Driver setup
    call InitFATDriver

    ;
    ; Find first cluster of bootable file
    ;
    call SearchFATDIR
    push dword eax

    lea ax, [FileFound_OK_cstr]
    push ax
    call PrintString
    add sp, 0x2

    push dword eax
    call PrintDWORD     ; void PrintDWORD(uint32_t dword)
    add sp, 0x4

    lea ax, [NewLine_cstr]
    push ax
    call PrintString
    add sp, 0x2
    
    ERROR STEVIA_DEBUG_HALT

hcf:
    hlt
    jmp short hcf

; ###############
;
; FAT32 Driver
;
; ###############

%include 'fat32/fat32_func_old.inc'

; ###############
;
; BIOS functions
;
; ###############

%include 'BIOS/BIOS_sys.inc'

; ##############################
;
; SYSTEM CONFIGURATION FUNCTIONS
;
; ##############################

; Prints a C-Style string (null terminated) using BIOS vga teletype call
; void PrintString(char* buf)
PrintString:
    __CDECL16_ENTRY
    mov di, [bp + 4]    ; first arg is char* 
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
    push ax
    call PrintCharacter
    add sp, 0x2

    inc si
    dec cx

    jcxz PrintString.endp
    jmp PrintString.print_L0    ; Fetch next character from string
.endp:
    __CDECL16_EXIT
    ret                         ; Return from procedure

; Prints a single character
; void PrintCharacter(char c);
PrintCharacter:
    __CDECL16_ENTRY
.func:
    mov ax, [bp + 4] ; c
    mov dx, 0x00ff
    and ax, dx

    mov ah, 0x0E                ; INT 0x10, AH=0x0E call
    mov bx, 0x0007              ; BH = page no. BL =Text attribute 0x07 is lightgrey font on black background
    int 0x10                    ; call video interrupt
.endp:
    __CDECL16_EXIT
    ret

; TODO: fix the prolog, epilog and stack usage to confirm with cdecl16
; prints the hex representation of of val
; void PrintDWORD(uint32_t val);
PrintDWORD:
    __CDECL16_ENTRY
.func: 
    lea si, [IntToHex_table]
    mov ebx, 16     ; base-16

    mov dword eax, [bp + 4]     ;val

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
    __CDECL16_EXIT
    ret

; ##############################
;
; SYSTEM CONFIGURATION FUNCTIONS
;
; ##############################

EnterUnrealMode:
    __CDECL16_ENTRY
.func:
    cli                         ; no interrupts
    push ds                     ; save real mode data selector
    lgdt [unreal_gdt_info]

    mov  eax, cr0               ; switch to pmode
    or al,1                     ; set pmode bit
    mov  cr0, eax

    jmp $+2                     ; clear instruction cache

    mov  bx, 0x08               ; select descriptor 1
    mov  ds, bx                 ; 8h = 1000b

    and al,0xFE                 ; back to realmode
    mov  cr0, eax               ; by toggling bit again

    pop ds                      ; get back old segment
    sti
.endp:
    __CDECL16_EXIT
    ret

; #############
;
; Strings
;
; #############

%define StrCRLF_NUL 0Dh, 0Ah, 00h

HelloPrompt_cstr:
    db 'Stevia Stage2', StrCRLF_NUL
A20_Enabled_cstr:
    db 'A20 Enabled OK', StrCRLF_NUL
MemoryMap_OK_cstr:
    db 'Memory map OK', StrCRLF_NUL
UnrealMode_OK_cstr:
    db 'Unreal mode OK', StrCRLF_NUL
FileFound_OK_cstr:
    db 'Found SFN entry for bootable binary, first cluster -> ', 00h
IntToHex_table:
    db '0123456789ABCDEF'
NewLine_cstr:
    db StrCRLF_NUL
BootTarget_str:
    db "BOOTI686BIN"

; #############
;
; Locals
;
; #############

partition_offset:
    dw 0x0000



; GDT documentation below:
;
;    Pr: Present bit. This must be 1 for all valid selectors.
;
;    Privl: Privilege, 2 bits. Contains the ring level,
;           0 = highest (kernel), 3 = lowest (user applications).
;
;    S: Descriptor type. This bit should be set for code or data segments
;       and should be cleared for system segments (eg. a Task State Segment)
;
;    Ex: Executable bit. If 1 code in this segment can be executed
;        ie. a code selector. If 0 it is a data selector.
;
;    DC: Direction bit/Conforming bit.
;        Direction bit for data selectors: Tells the direction.
;        0 the segment grows up. 1 the segment grows down, ie. the offset has to be greater than the limit.
;
;        Conforming bit for code selectors:
;            If 1 code in this segment can be executed from an equal or lower privilege level.
;               For example, code in ring 3 can far-jump to conforming code in a ring 2 segment.
;               The privl-bits represent the highest privilege level that is allowed to execute the segment.
;                   For example, code in ring 0 cannot far-jump to a conforming code segment with privl==0x2
;                   while code in ring 2 and 3 can. Note that the privilege level remains the same
;                   ie. a far-jump form ring 3 to a privl==2-segment remains in ring 3 after the jump.
;
;            If 0 code in this segment can only be executed from the ring set in privl.
;
;    RW: Readable bit/Writable bit.
;        Readable bit for code selectors: Whether read access for this segment is allowed. Write access is never allowed for code segments.
;        Writable bit for data selectors: Whether write access for this segment is allowed. Read access is always allowed for data segments.
;
;    Ac: Accessed bit. Just set to 0. The CPU sets this to 1 when the segment is accessed.
;
;    Gr: Granularity bit. If 0 the limit is in 1 B blocks (byte granularity), if 1 the limit is in 4 KiB blocks (page granularity).
;
;    Sz: Size bit. If 0 the selector defines 16 bit protected mode. If 1 it defines 32 bit protected mode.
;        You can have both 16 bit and 32 bit selectors at once.
;
;    AvL: Availible to software bit, the CPU does not use this field and software can read/write to it
;
;    D/B bit: The default operand-size bit is found in code-segment and data-segment descriptors but not in system-segment descriptors. Setting
;              this bit to 1 indicates a 32-bit default operand size, and clearing it indicates a 16-bit default size.
;
;    E bit: Expand down bit: Setting this bit to 1 identifies the data segment as expand-down.
;           In expand-down segments, the segment limit defines the lower segment boundary while the base is the upper boundary
;
; A GDT entry is 8 bytes and is constructed as follows:
; First DWORD
;   0-15	Limit 0:15	First 16 bits in the segment limiter
;   16-31	Base 0:15	First 16 bits in the base address
;
; 2nd DWORD
;
;   0:7	    Base 16:23	Bits 16-23 in the base address
;   8:12	S/Type	    Segment type and attributes, S = bit 12, Type = 8:11, Type is either [1, DC, RW, Ac] <code> or [0, E, RW, Ac] <data>
;   13:14	Privl	    0 = Highest privilege (OS), 3 = Lowest privilege (User applications)
;   15	    Pr	        Set to 1 if segment is present
;   16:19	Limit 16:19	Bits 16-19 in the segment limiter
;   20:22	Attributes	Different attributes, depending on the segment type
;   23	    Gr	        Used together with the limiter, to determine the size of the segment
;   24:31	Base 24:31	The last 24-31 bits in the base address
;
;
;

ALIGN 4, db 0
unreal_gdt_info:
    unreal_gdt_size: dw (unreal_gdt_end - unreal_gdt_start) - 1
    unreal_gdt_ptr:  dd unreal_gdt_start

unreal_gdt_start:
    ; entry 0
    dq 0                    ; first entry is null

    ; entry 1 (4 GiB flat data map)
    ; first dword
    dw 0xFFFF               ; 0:15 limit
    dw 0x0000               ; 0:15 base
    ; second dword
    db 0x00                 ; 16:23 base
    db 1001_0010b            ; bit 0:4 = S/Type, [1, DC, RW, Ac] <code> or [0, E, RW, Ac] <data>
                            ; bit 5:6 = Privl
                            ; bit 7   = Present

    db 1000_1111b            ; bit 0:3 = 16:19 of Limit
                            ; bit 4 = Availible to software bit
                            ; bit 5 = Reserved (?)
                            ; bit 6 = D/B bit, depending on if this is code/data 1 = 32 bit operands or stack size
                            ; bit 7 = Granularity bit. 1 = multiply limit by 4096
    db 0x00                 ; base 24:31
    ; at the end of the day...
    ; base = 0x00000000
    ; limit = 0xFFFFF
    ; Accessed = 0, ignore this field
    ; RW = 1, data is Read/Write
    ; E = 0, Expand up, valid data is from base -> limit, if 1 valid data is from (limit + 1) -> base
    ; C/D = 0, Segment is a data segment
    ; S = 1, Segment is a system segment
    ; Privl = 00b, Ring0 segment
    ; Pr = 1, segment is present
    ; AVL = 0, ignore this field
    ; D/B bit = 0, 16bit code/stack
    ; Gr = 1, multiply limit by 4096
unreal_gdt_end:

ALIGN 4, db 0
gdt32_info:
    gdt32_size: dw (gdt32_end - gdt32_start) - 1
    gdt32_ptr:  dd gdt32_start

; check above for detailed information
gdt32_start:
    dq 0
    .gdt32_code:
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_1000b
        db 1100_1111b
        db 0x00
    .gdt32_data:
        dw 0xFFFF
        dw 0x0000
        db 0x00
        db 1001_0010b
        db 1100_1111b
        db 0x00
gdt32_end:

%assign bytes_remaining ((MAX_STAGE2_BYTES - 4) - ($ - $$))
%warning STAGE2 has bytes_remaining bytes remaining for code (MAX: MAX_STAGE2_BYTES)

; this is here to make the stage2 bigger than 1 sector for testing
times ((MAX_STAGE2_BYTES - 4) - ($ - $$)) db 0x00
STAGE2_SIG: dd 0xDEADBEEF