target remote localhost:1234
set architecture i386
set disassembly-flavor intel

display/x $eip
display/x $esp
display/x $ebp

graph display `x /16xw ($ebp - (16*4))`
