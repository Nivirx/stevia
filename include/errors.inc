;    Copyright (C) 2020  Bradley Claus
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


; Errors
; 12 Errors, 5 in use
%define MBR_ERROR_DISK_T_ERR         'a'
%define MBR_ERROR_NO_INT32E          'b'
%define MBR_ERROR_NO_NO_BOOT_PART    'c'
%define MBR_ERROR_DISK_READ_ERR      'd'
%define MBR_ERROR_NO_VBR_SIG         'e'
%define MBR_ERROR_RESERVED_f         'f'
%define MBR_ERROR_RESERVED_g         'g'
%define MBR_ERROR_RESERVED_h         'h'
%define MBR_ERROR_RESERVED_i         'i'
%define MBR_ERROR_RESERVED_j         'j'
%define MBR_ERROR_RESERVED_k         'k'
%define MBR_ERROR_RESERVED_l         'l'

; 12 Error
%define VBR_ERROR_WRONG_FAT_SIZE     'm'
%define VBR_ERROR_NO_SIGNATURE       'n'
%define VBR_ERROR_DISK_READ_ERR      'o'
%define VBR_ERROR_RESERVED_p         'p'
%define VBR_ERROR_RESERVED_q         'q'
%define VBR_ERROR_RESERVED_r         'r'
%define VBR_ERROR_RESERVED_s         's'
%define VBR_ERROR_RESERVED_t         't'
%define VBR_ERROR_RESERVED_u         'u'
%define VBR_ERROR_RESERVED_v         'v'
%define VBR_ERROR_RESERVED_w         'w'
%define VBR_ERROR_RESERVED_x         'x'

; 26 errors, 8 in use
%define STAGE2_A20_FAILED            'A'
%define STAGE2_SIGNATURE_MISSING     'B'
%define STAGE2_MM_E820_NO_SUPPORT    'C'
%define STAGE2_MM_E820_MISC_ERR      'D'
%define STAGE2_MM_E820_NONSTANDARD   'E'
%define STAGE2_MM_E820_NO_SMAP       'F'
%define STAGE2_MBR_DISK_READ_ERROR   'G'
%define STAGE2_FAT32_INIT_ERROR      'H'
%define STAGE2_FAT32_NO_FILE         'I'
%define STAGE2_FAT32_END_OF_CHAIN    'J'
%define STAGE2_ERROR_RESERVED_K      'K'
%define STAGE2_ERROR_RESERVED_L      'L'
%define STAGE2_ERROR_RESERVED_M      'M'
%define STAGE2_ERROR_RESERVED_N      'N'
%define STAGE2_ERROR_RESERVED_O      'O'
%define STAGE2_ERROR_RESERVED_P      'P'
%define STAGE2_ERROR_RESERVED_Q      'Q'
%define STAGE2_ERROR_RESERVED_R      'R'
%define STAGE2_ERROR_RESERVED_S      'S'
%define STAGE2_ERROR_RESERVED_T      'T'
%define STAGE2_ERROR_RESERVED_U      'U'
%define STAGE2_ERROR_RESERVED_V      'V'
%define STAGE2_ERROR_RESERVED_W      'W'
%define STAGE2_ERROR_RESERVED_X      'X'
%define STAGE2_ERROR_RESERVED_Y      'Y'
%define STAGE2_ERROR_RESERVED_Z      'Z'


%macro ERROR 1
    mov al, %1
    jmp error
%endmacro

; pass error as ascii character in al, errors a-zA-Z or 0-9
error:
    ; color 0x4F is white on red
    ; fs = 0xb800 => fs:0x0000 = 0xb8000
    mov dx, 0xB800
    mov fs, dx

    ; the characters are two bytes in the order of 0xb8000: byte c, byte attribute
    ; since x86 is le, we store the attribute in the MSB of dx
    mov dh, 0x4F
    mov dl, al
    mov word [fs:0x0000], dx

.stop:
    hlt
    jmp short error.stop