set tdesc filename target.xml
set arch i8086
set disassembly-flavor intel
target remote localhost:1234

display/i $cs*16+$pc
b *0x7c00      
set $cs = 0xf000
set $pc = 0xfff0
c
