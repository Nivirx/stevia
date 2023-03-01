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


; 8KiB from 0x2500 -> 0x500
%define STACK_START            0x2500
%define MBR_ENTRY              0x7A00
%define VBR_ENTRY              0x7C00
%define STAGE2_ENTRY           0x7E00