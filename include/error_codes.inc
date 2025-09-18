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

%ifndef __INC_ERROR_CODES

; MBR Error codes
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
%define MBR_ERROR_INT13h_EREAD_ERR   'l'

; VBR Error codes
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

; Stage2 Error codes
%define STAGE2_A20_FAILED            'A'
%define STAGE2_SIGNATURE_MISSING     'B'
%define STAGE2_MM_E820_NO_SUPPORT    'C'
%define STAGE2_MM_E820_MISC_ERR      'D'
%define STAGE2_MM_E820_NONSTANDARD   'E'
%define STAGE2_MM_E820_NO_SMAP       'F'
%define STAGE2_INT13_DISK_READ_ERR   'G'
%define STAGE2_FAT32_E_FMT           'H'
%define STAGE2_FAT32_E_RANGE         'I'
%define STAGE2_FAT32_E_NOSUCH        'J'
%define STAGE2_FAT32_E_ISDIR         'K'
%define STAGE2_FAT32_E_TOOLONG       'L'
%define STAGE2_FAT32_E_UNSUPPORTED   'M'
%define STAGE2_FAT32_E_UNIMPLEMENTED 'N'
%define STAGE2_ERROR_BAD_MBR         'O'
%define STAGE2_VBR_E_ACTIVE          'P'
%define STAGE2_VBR_E_SIGN            'Q'
%define STAGE2_VBR_E_TOT             'R'
%define STAGE2_VBR_E_FATSZ           'S'
%define STAGE2_VBR_E_DIRENT          'T'
%define STAGE2_VBR_E_PARTTYPE        'U'
%define STAGE2_RESERVED_E_V          'V'

; Debug error codes
%define STEVIA_DEBUG_OK              'W'
%define STEVIA_DEBUG_ERR             'X'
%define STEVIA_DEBUG_UNIMPLEMENTED   'Y'
%define STEVIA_DEBUG_HALT            'Z'

%endif
%define __INC_ERROR_CODES