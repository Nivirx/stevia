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

%ifndef __INC_ERROR_CODES

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
%define MBR_ERROR_INT13h_EREAD_ERR   'l'

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

; 22 errors, 8 in use
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

; for development only, specific errors should be above.
%define STEVIA_DEBUG_OK      'W'
%define STEVIA_DEBUG_ERR      'X'
%define STEVIA_DEBUG_UNIMPLEMENTED      'Y'
%define STEVIA_DEBUG_HALT      'Z'

%endif
%define __INC_ERROR_CODES