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

%ifndef __INC_EARLY_MEM

;PhysicalAddress = Segment * 16 + Offset
%define SEG_TO_LINEAR(s,o) ((s << 4) + o)

; Offset = physical / (Segment * 16)
%define LINEAR_TO_OFFSET(p,s) ((p / (s << 4)))

; Seg = (physical - offset) / 16
%define LINEAR_TO_SEGMENT(p,o) ((p - o) >> 4)

%endif
%define __INC_EARLY_MEM