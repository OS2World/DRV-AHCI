; Thunking for ahci32 driver
; Copyright (c) 2018 David Azarewicz
; Author: David Azarewicz <david@88watts.net>

                EXTRN Dos32FlatDS:ABS

;==========================================================
; 16 bit code segment
;
_TEXT16 SEGMENT byte PUBLIC USE16 'CODE'
        assume ds:nothing, es:nothing
IoctlWakeup16 Proc far
        jmp     far ptr FLAT:IoctlWakeup32
IoctlWakeup16 endp

_TEXT16 ENDS

;==========================================================
; 32 bit data segment
;
_DATA     segment dword public use32 'DATA'

Far16AdrOfIoctlWakeup16 dw offset _TEXT16:IoctlWakeup16
                        dw seg _TEXT16:IoctlWakeup16
        public Far16AdrOfIoctlWakeup16

_DATA       ends

;==========================================================
; 32 bit code segment
;
_TEXT   segment byte public use32 'CODE'
        assume cs:FLAT

        EXTRN KernThunkStackTo16:near
        EXTRN D32ThunkStackTo32:near
        EXTRN IoctlWakeup:near

IoctlWakeup32 Proc far
        push    ebp
        mov     ebp, esp
        push    ds
        push    es
        push    ebx
        push    esi
        push    edi

        ; load FLAT selector into ES and DS
        mov     eax, offset Dos32FlatDS
        mov     es, eax
        mov     ds, eax
        ASSUME ds:FLAT,es:FLAT

        call D32ThunkStackTo32
        ASSUME ss:FLAT

        mov     eax, [ebp+8]        ; Parameter pointer
        push    eax
        call    IoctlWakeup
        add     esp,4
        mov     ebx, eax

        call    KernThunkStackTo16
        mov     eax, ebx

        pop     edi
        pop     esi
        pop     ebx
        pop     es
        pop     ds
        pop     ebp

        db      66h                 ; force 16 bit return
        retf
IoctlWakeup32 EndP

_TEXT   ends

        end
