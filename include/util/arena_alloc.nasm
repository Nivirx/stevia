%ifndef __INC_ARENA_ALLOC_FUNC

%define __ARENA_HEAP_START 0x8000
%define __ARENA_HEAP_SIZE  0x6000   ; 0x8000 -> 0xE000 = 24KiB of Heap

struc ArenaStateStruc_t
    .start resw 1
    .end resw 1
    .mark resw 1
endstruc

; void arena_init(ArenaState *a)
;
arena_init:
    __CDECL16_ENTRY
.func:
    movzx eax, word [bp + 4]     ; ptr to state structure
    mov di, ax

    xor eax, eax
    mov word [di + ArenaStateStruc_t.mark], ax
    mov word [di + ArenaStateStruc_t.end], word (__ARENA_HEAP_START + __ARENA_HEAP_SIZE)
    mov word [di + ArenaStateStruc_t.start], __ARENA_HEAP_START

    ; zero out heap area on init
    ; void* kmemset_byte(void* dst, uint8_t val, uint16_t len);
    ; TODO: use word or qword spacing at least to speed this up
    mov ax, __ARENA_HEAP_SIZE
    push ax                     ; len
    xor ax, ax
    push ax                     ; val = 0
    mov ax, __ARENA_HEAP_START
    push ax                     ; dst
    call kmemset
    add sp, 0x6

.endp:
    __CDECL16_EXIT
    ret

; size_t align_up(size_t x, size_t a)
; ax, bx. cx are all clobbered
; align x up to the nearest specified alignment (a), a should be a power of 2
; (x + (a-1)) & ~(a-1)
; return value in ax
arena_align_up:
    __CDECL16_ENTRY
.func:
    
    mov ax, [bp + 4]      ; x

    mov bx, [bp + 6]      
    sub bx, 1             ; a - 1

    mov cx, ax
    add cx, bx            ; x + (a+1)
    not bx                ; ~(a-1)

    and cx, bx            ; and with the inverse

    mov ax, cx            ; move to ax and return
.endp:
    __CDECL16_EXIT
    ret

; void* arena_alloc(void* a, size_t bytes, size_t align)
arena_alloc:
    __CDECL16_ENTRY
.func:
    ; remove bytes from pool and increment mark to the new cursor location
    ; return a pointer to the begining of  allocated segment
    ERROR STEVIA_DEBUG_UNIMPLEMENTED
.endp:
    __CDECL16_EXIT
    ret

arena_mark:
    __CDECL16_ENTRY
.func:
    ; return the current location of the 'cursor' in the allocator
    ERROR STEVIA_DEBUG_UNIMPLEMENTED
.endp:
    __CDECL16_EXIT
    ret

arena_reset_to:
    __CDECL16_ENTRY
.func:
    ; rewind the arena to a previously marked point
    ERROR STEVIA_DEBUG_UNIMPLEMENTED
.endp:
    __CDECL16_EXIT
    ret

arena_reset:
    __CDECL16_ENTRY
.func:
    ; reset the entire heap to a fresh state
    ERROR STEVIA_DEBUG_UNIMPLEMENTED
.endp:
    __CDECL16_EXIT
    ret

%endif
%define __INC_ARENA_ALLOC_FUNC