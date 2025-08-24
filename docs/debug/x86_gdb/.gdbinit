target remote localhost:1234
set architecture i386
set disassembly-flavor intel
layout asm

display/x $eip
display/x $esp
display/x $ebp
