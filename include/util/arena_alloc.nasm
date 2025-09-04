%ifndef __INC_ARENA_ALLOC_FUNC

%define __ARENA_HEAP_START 0x8000
%define __ARENA_HEAP_SIZE  0x6000   ; 0x8000 -> 0xE000 = 24KiB of Heap

struc ArenaStateStruc_t
    .start resw 1
    .end resw 1
    .mark resw 1
endstruc

; void arena_init(void *mem, size_t bytes)
;
arena_init:
    __CDECL16_ENTRY
.func:

.endp:
    __CDECL16_EXIT
    ret

; size_t align_up(size_t x, size_t a)
arena_align_up:
    __CDECL16_ENTRY
.func:
    ; align the mark to the nearest specified alignment
.endp:
    __CDECL16_EXIT
    ret

; void* arena_alloc(void* a, size_t bytes, size_t align)
arena_alloc:
    __CDECL16_ENTRY
.func:
    ; remove bytes from pool and increment mark to the new cursor location
    ; return a pointer to the begining of  allocated segment
.endp:
    __CDECL16_EXIT
    ret

arena_mark:
    __CDECL16_ENTRY
.func:
    ; return the current location of the 'cursor' in the allocator
.endp:
    __CDECL16_EXIT
    ret

arena_reset_to:
    __CDECL16_ENTRY
.func:
    ; rewind the arena to a previously marked point
.endp:
    __CDECL16_EXIT
    ret

arena_reset:
    __CDECL16_ENTRY
.func:
    ; reset the entire heap to a fresh state
.endp:
    __CDECL16_EXIT
    ret

%endif
%define __INC_ARENA_ALLOC_FUNC