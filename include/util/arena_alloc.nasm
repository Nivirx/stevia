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
    mov word [di + ArenaStateStruc_t.mark], eax
    mov word [di + ArenaStateStruc_t.end], word (__ARENA_HEAP_START + __ARENA_HEAP_SIZE)
    mov word [di + ArenaStateStruc_t.start], __ARENA_HEAP_START

    ; zero out heap area on init
    ; void* kmemset_byte(void* dst, uint8_t val, uint16_t len);
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
; eax, ebx. ecx are all clobbered
arena_align_up:
    __CDECL16_ENTRY
.func:
    ; align x up to the nearest specified alignment (a)
    ; (x + (a-1)) & ~(a-1)
    mov eax, [bp + 4]   ; x

    mov ebx, [bp + 6]   ; a - 1
    sub ebx, 1

    mov ecx, eax
    add ecx, ebx        ; x + (a+1)

    not ebx
    and ecx, ebx        ; ecx contains the result

    xor eax, eax
    xor ebx, ebx
    mov eax, ecx        ; return result in eax
    xor ecx, ecx

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