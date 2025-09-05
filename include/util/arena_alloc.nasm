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
    mov ax, word [bp + 4]     ; ptr to state structure
    mov di, ax

    xor ax, ax
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
    ; if a == 0 return x
    mov ax, [bp + 4]      ; x
    mov bx, [bp + 6]      ; a

    or bx, bx
    jz .endp              

    ; enforce power-of-two for alignment, return x
    mov cx, bx
    dec cx
    test bx, cx
    jnz .endp

    dec bx                ; a - 1

    mov cx, ax
    add cx, bx            ; x + (a-1)
    not bx                ; ~(a-1)

    and cx, bx            ; and with the inverse

    mov ax, cx            ; move to ax and return
.endp:
    __CDECL16_EXIT
    ret

; void* arena_alloc(size_t bytes, size_t align)
; bp-2 - current used arena (i.e 'highmark')
; bp-4 - aligned_ptr 
; bp-6 - new_end
arena_alloc:
    __CDECL16_ENTRY 0x10
.func:
    ; remove bytes from pool and increment mark to the new cursor location
    ; return a pointer to the begining of  allocated segment
    mov bx, early_heap_state
    
    mov ax, word [bx + ArenaStateStruc_t.start]
    mov dx, word [bx + ArenaStateStruc_t.mark]
    add ax, dx                      ; i.e the total used arena
    mov word [bp - 2], ax           ; save as a local

    push bx                         ; save heap_state pointer

    mov ax, word [bp + 6]           ; requested next allocation alignment
    push ax
    mov ax, word [bp - 2]           ; current arena 'highmark'
    push ax
    call arena_align_up
    add sp, 0x4
    mov word [bp - 4], ax           ; save return value

    pop bx                          ; restore heap_state pointer

    ; new_end = aligned_ptr + bytes 
    add ax, word [bp + 4]           ; add to total (so current aligned mark + bytes)
    mov word [bp - 6], ax           ; save as local

    mov dx, word [bx + ArenaStateStruc_t.end]
    cmp ax, dx
    ja arena_alloc.error            ; if our heap end is < the requested throw an error, heap is full
                                    ; else update the mark to the new value & return  the aligned pointer

    ; mark_delta = new_end - curr_used
    mov dx, word [bp - 6]
    sub dx, word [bp - 2]

    ; mark += mark_delta
    mov ax, word [bx + ArenaStateStruc_t.mark]
    add ax, dx
    mov word [bx + ArenaStateStruc_t.mark], ax
    
    ; return aligned_ptr
    mov ax, word [bp - 4]
.endp:
    __CDECL16_EXIT
    ret
.error:
    ERROR STEVIA_DEBUG_ERR

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