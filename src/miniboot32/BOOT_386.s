;    Copyright (C) 2023  Elaina Claus
;
;    This program is free software: you can redistribute it and/or modify
;    it under the terms of the GNU General Public License as published by
;    the Free Software Foundation, either version 3 of the License, or
;    (at your option) any later version.
;
;    This program is distributed in the hope that it will be useful,
;    but WITHOUT ANY WARRANTY; without even the implied warranty of
;    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
;    GNU General Public License for more details.
;
;    You should have received a copy of the GNU General Public License
;    along with this program.  If not, see <https://www.gnu.org/licenses/>.

[BITS 32]
[ORG 0x100000]
[CPU KATMAI]

jmp short start32
nop

; Unless noted otherwise, functions will be the standard x86 cdecl used in GCC

; In cdecl, subroutine arguments are passed on the stack. 
; Integer values and memory addresses are returned in the EAX register,
; floating point values in the ST0 x87 register. 
; Registers EAX, ECX, and EDX are caller-saved, 
; and the rest are callee-saved. 
; The x87 floating point registers ST0 to ST7 must be empty (popped or freed) 
; when calling a new function, and ST1 to ST7 must be empty on exiting a function. 
; ST0 must also be empty when not used for returning a value. 
;
;   EXAMPLE: to call uint8_t* kmemset(uint8_t* dest, uint8_t val, uint8_t size);
;               => kmemset(*((char*) 0xb8000), 'F', (80*25))
;
;   push ebp                ; save old call frame pointer  
;   mov ebp, esp            ; new call frame pointer
;   push 0x000007D0         ; push args RTL, EBP - 4
;   push 'F'                ; *              EBP - 8
;   push 0x000b8000         ; *              EBP - 12
;   call kmemset            ; call function, this also places a return pointer on stack
;   add esp, 12             ; remove call arguments from stack frame
;   mov esp, ebp            ; restore stack frame pointer (callee saves EBP)
;   pop ebp                 ; restore call frame pointer
;
;

; 33 bytes BPB + 26 Byte EBPB
struc BPBStruct
    .OemName        resb 8
    .BytesPerSect   resw 1
    .SecsPerClust   resb 1
    .ResSectors     resw 1
    .FATs           resb 1
    .RootDirEnts    resw 1
    .Sectors        resw 1
    .Media          resb 1
    .SectPerFAT     resw 1
    .SectPerTrack   resw 1
    .Heads          resw 1
    .Hidden         resd 1
    .SectorHuge     resd 1
    ; begin EBPB
    .DriveNumber    resb 1
    .NTReserved     resb 1
    .Signature      resb 1
    .VolumeID       resd 1
    .VolumeLabel    resb 11
    .SysIdent       resb 8
endstruc

; 12 bytes
struc FSInfoStruct
    .first_root_dir_sector resw 1
    .last_root_dir_sector  resw 1
    .root_dir_len          resw 1
    .first_data_sector     resw 1
    .active_cluster        resw 1
    .active_FAT_cluster    resw 1
endstruc

; 20 bytes
struc KInfoStruct
    .load_address          resd 1
    .file_len              resd 1
    .file_name             resb 8
    .file_ext              resb 3
    .reserved1             resb 1
endstruc

; Address Range Descriptor Structure
;
; Offset in Bytes		Name		Description
;	0	    BaseAddrLow		u32 - Low 32 Bits of Base Address
;	4	    BaseAddrHigh	u32 - High 32 Bits of Base Address
;	8	    LengthLow		u32 - Low 32 Bits of Length in Bytes
;	12	    LengthHigh		u32 - High 32 Bits of Length in Bytes
;	16	    Type		    u32 - Address type of  this range.
;   20      ExtType         u32 -  ACPI 3.0 extended type
struc AddressRangeDescStruct
    .BaseAddrLow    resd 1
    .BaseAddrHigh   resd 1
    .LengthLow      resd 1
    .LengthHigh     resd 1
    .Type           resd 1
    .ExtType        resd 1
endstruc

; 20 bytes, passed to loaded kernel
struc CBootInfoStruct
    .MemoryMapPtr      resd 1
    .MemoryMapEntries  resd 1
    .BPBDataPtr        resd 1
    .FSInfoPtr         resd 1
    .KInfoPtr          resd 1
endstruc

;;;
; Errors
;   0 = unused
;   1 = No CPUID support on this platform (VM or some very old hardware?)
;   2 = magic signature not found at end of file
;   3 = Unexpected/unhandled artithmetic overflow/carry
;;;

%define MAX_BYTES (1024 * 8)

%define VGA_BUF 0xb8000
;black BG/Green FG
%define TTY_COLOR 0x0200
;black BG/Red FG
%define TTY_ERR_CLR 0x0400
;subtract 1 for max array values
%define VGA_MAX_Y 25
%define VGA_MAX_X 80

; VGA memory is row-major => offset = (row * MAX_Colums) + colum
; macro array is counted from 0,0 = 1,1 => 79,24 is the last usable space in a page
%define VGA_OFFSET(x,y) (((y*VGA_MAX_X) + x)*2)

start32:
    mov ax, 0x10
    mov ds, ax
    mov es, ax
    mov fs, ax
    mov gs, ax
    mov ss, ax          ; load data registers with 2nd GDT selector
    mov esp, stack_top
    mov ebp, esp

; debugger hack because gdb doesn't like to switch register sizes
; issue a set $eax = 1 to continue
%ifdef DEBUG_HACK
    mov eax, 0
.gdb_hack:
    test eax, eax
    jz start32.gdb_hack
%endif

    mov eax, dword [BOOT_SIG]
    cmp eax, 0xA0B0C0D0
    jz start32.signature_present

    mov al, "2"
    call error


.signature_present:
    call check_cpuid

    push ebp
    mov ebp, esp
    lea eax, [welcome_cstr]
    push eax
    call vga_puts
    leave

    push ebp
    mov ebp, esp
    lea eax, [version_cstr]
    push eax
    call vga_puts
    leave

    push ebp
    mov ebp, esp
    lea eax, [datetime_cstr]
    push eax
    call vga_puts
    leave

    mov eax, VGA_BUF
    add eax, VGA_OFFSET(78, 24)
    mov dword [eax], 0x2f4b2f4f
    hlt

; Early error printer. Prints 'ERR: X' where X is an error code in al
error:
    mov eax, VGA_BUF
    add eax, VGA_OFFSET(73, 24)

    mov dword [eax], 0x4f524f45
    add eax, 0x2
    mov dword [eax], 0x4f3a4f52
    add eax, 0x2
    mov dword [eax], 0x4f204f20
    add eax, 0x2
    mov byte  [eax], al
    hlt

check_cpuid:
    ; flip cpuid id bit (bit 21) to check for cpuid support
    
    ; copy flags to eax via stack
    pushfd
    pop eax

    mov ecx, eax        ; copy to ecx for compare

    xor eax, 1 << 21    ; xor eax with the 21st bit set

    push eax            ; push new flags and pop to flags register
    popfd

    pushfd
    pop eax

    cmp eax, ecx        ; can we change the bit?
    je .no_cpuid
    ret
.no_cpuid:
    mov al, "1"
    call error

; int vga_puts(char* str)
;
; INPUT: 
;   str: C-Sytle string
;
; OUTPUT:
;   EAX = characters printed
;
vga_puts:
    ; prolog, EBP = SFP (Stack frame pointer), [EBP-4] = char* str
    push edi
    push esi
    push ebx

.get_str_len:
    mov edi, [ebp - 4]

    xor ecx, ecx        ; ECX = 0
    not ecx             ; ECX = -1 == 0xFFFFFFFF
    xor eax, eax        ; search for al = 0x0
    cld
    repne scasb         ; deincrement ecx while searching for al
    not ecx             ; the inverse of a neg number = abs(x) - 1
    dec ecx             ; ECX contains the length of the string - nul byte at end


    mov edi, VGA_BUF    ; load dest index with VGA buffer + calculated offset
    mov esi, [ebp -4]   ; load source index with src string

    ; ECX = string length
    ; ESI = source string to print
    ; EDI = destination in the VGA buffer
.print_char:
    cmp ecx, 0x0
    jle vga_puts.endf

    cmp byte [vga_buf_pos_x], 80
    je vga_puts.end_of_line

    cmp byte [vga_buf_pos_y], 25
    je vga_puts.scroll_up

    cmp byte [esi], 0x20
    jl vga_puts.special_character_switch

    push ebp
    mov ebp, esp

    xor eax, eax
    mov al, [vga_buf_pos_y]
    push eax                ; vga_pos_y

    xor eax, eax
    mov al, [vga_buf_pos_x]
    push eax                ; vga_pos_x

    ; uint16_t vga_calc_offset(uint8_t vga_pos_x, uint8_t vga_pos_y)
    call vga_calc_offset
    leave

    xor bx, bx
    mov bl, byte [esi]
    or bx, TTY_COLOR            ; create VGA character

    mov [edi+eax], bx
    inc esi
    inc byte [vga_buf_pos_x]

    
    dec ecx
    jmp vga_puts.print_char    ; loop until ECX == 0

.end_of_line:
    ; advance to next row
    inc byte [vga_buf_pos_y]
    mov byte [vga_buf_pos_x], 0
    jmp vga_puts.print_char
.scroll_up:
    ; after 25 rows, move to next page (either swap pages [memcpy] or erase page 0)
    mov byte [vga_buf_pos_x], 0
    mov byte [vga_buf_pos_y], 0
    
    push ebp
    mov ebp, esp

    mov eax, 0x7D0
    push eax        ; 1 VGA page = 0x7D0 elements

    xor eax, eax
    push eax        ; we want to blank the page with 0x0000

    mov eax, VGA_BUF
    push eax        ; load vga buffer base (0xb8000)

    ; uint16_t* kmemset_word(void* dest, uint16_t val, size_t len);
    call kmemset_byte
    leave

    jmp vga_puts.print_char

.special_character_switch:

    cmp byte[esi], 0xA
    je vga_puts.handle_LF

    cmp byte[esi], 0xD
    je vga_puts.handle_CR

; Default case
.special_character_endp:
    dec ecx
    inc esi

    jmp vga_puts.print_char

.handle_LF:
    ; advance to next row
    inc byte [vga_buf_pos_y]
    jmp vga_puts.special_character_endp

.handle_CR:
    mov byte [vga_buf_pos_x], 0
    jmp vga_puts.special_character_endp


.endf:
    pop ebx
    pop esi
    pop edi

    ret

; short vga_get_xy(void)
;
; INPUT: 
;
; OUTPUT:
;   EAX = xy_pos
;      > x = xy_pos && 0xFF00
;        y = xy_pos && 0x00FF
;   2 MSB of EAX are 0.
;
vga_get_xy:
.endf:
    ret

; void vga_set_xy(short xy_pos)
;   xy_pos is a byte packed word (16bit value), MSB is x, LSB is y
;       => x = xy_pos && 0xFF00
;          y = xy_pos && 0x00FF
;
; INPUT: 
;   xy_pos
;
; OUTPUT:
;   NONE
;
vga_set_xy:
.endf:
    ret

; uint16_t vga_calc_offset(uint8_t vga_pos_x, uint8_t vga_pos_y)
; VGA_OFFSET(x,y) == (((y*VGA_MAX_X) + x)*2)
vga_calc_offset:
    xor eax, eax
    xor edx, edx

    mov edx, [ebp-4]
    mov ax, VGA_MAX_X
    mul dx              ; DX:AX contains (y*VGA_MAX_X)
    jc vga_calc_offset.artithmetic_error

    mov edx, [ebp-8]
    add ax, dx
    jc vga_calc_offset.artithmetic_error

    mov dx, 0x02
    mul dx              ; DX:AX contains (y*VGA_MAX_X) + x) * 2
    jnc vga_calc_offset.endf

.artithmetic_error:
    mov al, '3'
    call error

.endf:
    ; (e)AX contains the offset
    ret

; uint32_t* kmemset(void* dest, uint32_t val, size_t len);
kmemset_dword:
    push    edi             ; function uses edi, so save it.
                            ; [ebp] = previous call frame
    mov     ecx, [ebp - 4]  ; size_t len

    mov     eax, [ebp - 8]  ; uint32_t val

    mov     edi, [ebp - 12] ; void * ptr
                            ; [ebp - 16] = return adress
                            ; [ebp - 20] = saved EDI
    rep     stosd 
.endf:
    mov     eax, [ebp - 12] ; return pointer to dest
    pop     edi             ; restore edi
    ret

; uint16_t* kmemset(void* dest, uint16_t val, size_t len);
kmemset_word:
    push    edi             ; function uses edi, so save it.
                            ; [ebp] = previous call frame
    mov     ecx, [ebp - 4]  ; size_t len

    mov     ax, [ebp - 8]   ; uint16_t val

    mov     edi, [ebp - 12] ; void * ptr
                            ; [ebp - 16] = return adress
                            ; [ebp - 20] = saved EDI
    rep     stosw 
.endf:
    mov     eax, [ebp - 12] ; return pointer to dest
    pop     edi             ; restore edi
    ret

; uint8_t* kmemset(void* dest, uint8_t val, size_t len);
kmemset_byte:
    push    edi             ; function uses edi, so save it.
                            ; [ebp] = previous call frame
    mov     ecx, [ebp - 4]  ; size_t len

    mov     al, [ebp - 8]   ; uint8_t val

    mov     edi, [ebp - 12] ; void * ptr
                            ; [ebp - 16] = return adress
                            ; [ebp - 20] = saved EDI
    rep     stosb 
.endf:
    mov     eax, [ebp - 12] ; return pointer to dest
    pop     edi             ; restore edi
    ret

; uint32_t* kmemset(uint32_t* dest, uint32_t* src, size_t len);
kmemcpy_dword:
    push edi
    push esi                ; edi, esi are callee save

    mov edi, [ebp-12]       ; dest
    mov esi, [ebp-8]        ; source
    mov ecx, [ebp-4]        ; length

    cld                     ; ensure we are incrementing
    rep movsd

.endf:
    mov eax, [ebp-12]       ; return pointer to dest
    pop esi
    pop edi
    ret

; uint16_t* kmemset(uint16_t* dest, uint16_t* src, size_t len);
kmemcpy_word:
    push edi
    push esi                ; edi, esi are callee save

    mov edi, [ebp-12]       ; dest
    mov esi, [ebp-8]        ; source
    mov ecx, [ebp-4]        ; length

    cld                     ; ensure we are incrementing
    rep movsw

.endf:
    mov eax, [ebp-12]       ; return pointer to dest
    pop esi
    pop edi
    ret

; uint8_t* kmemset(uint8_t* dest, uint8_t* src, size_t len);
; not overlap safe
kmemcpy_byte:
    push edi
    push esi                ; edi, esi are callee save

    mov edi, [ebp-12]       ; dest
    mov esi, [ebp-8]        ; source
    mov ecx, [ebp-4]        ; length

    cld                     ; ensure we are incrementing
    rep movsb

.endf:
    mov eax, [ebp-12]       ; return pointer to dest
    pop esi
    pop edi
    ret

; Static Values
vga_buf_pos_x:
    db 0x00
vga_buf_pos_y:
    db 0x00

; Strings
%define StrCRLF_NUL 0Dh, 0Ah, 00h
welcome_cstr:
    db 'CBoot Stage3', StrCRLF_NUL
version_cstr:
    db 'CBoot v0.0.3 ', 'NASM - ', __NASM_VER__, StrCRLF_NUL
datetime_cstr:
    db 'Assembled - ', __DATE__, ' ', __TIME__, StrCRLF_NUL

ALIGN 16, db 0
stack_bottom:
    times (512 * 8) db 0x00       ; 4KiB stack
stack_top:

%assign bytes_remaining ((MAX_BYTES - 4) - ($ - $$))
%warning boot32 has bytes_remaining bytes remaining for code (MAX: MMAX_BYTES)

times ((MAX_BYTES - 4) - ($ - $$)) db 0xFE
BOOT_SIG: dd 0xA0B0C0D0