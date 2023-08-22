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


%define SECTOR_SIZE             512
%define STAGE2_SECTOR_COUNT    0x40
; 32 KiB
%define MAX_STAGE2_BYTES      (SECTOR_SIZE * STAGE2_SECTOR_COUNT)