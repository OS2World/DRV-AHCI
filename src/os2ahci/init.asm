; ----------------------------------------------------------------------------
; Initialization routines and segment mappings for os2ahci driver, plus
; some low-level functions that were implemented in assembler

include devhdr.inc

; -----------------------------------------------------------------------------
; Constants
                DEV_IDC  EQU    0400h        ;  IDC bit for ADD flags

; -----------------------------------------------------------------------------
; Public symbols

                PUBLIC  _asm_strat           ; low-level strategy routine
                PUBLIC  _asm_idc_entry       ; low-level IDC entry point
                PUBLIC  _asm_krnl_exit       ; low-level kernel exit routine
                PUBLIC  _readl               ; MMIO read (32 bits)
                PUBLIC  _writel              ; MMIO write (32 bits)
                PUBLIC  _memset              ; C memset() implementation
                PUBLIC  _memcpy              ; C memcpy() implementation
                PUBLIC  _restart_hook        ; port restart context hook
                PUBLIC  _reset_hook          ; port reset context hook
                PUBLIC  _engine_hook         ; engine trigger context hook
                PUBLIC  _dev_hdr             ; device driver header
                PUBLIC  __I4M                ; 32bit signed multiply routine
                PUBLIC  __U4M                ; 32bit unsigned multiply routine
                PUBLIC  __U4D                ; 32bit unsigned divide routine
                PUBLIC  __I4D                ; 32bit signed divide routine
                PUBLIC  _end_of_data         ; end of all data (label)
                PUBLIC  _end_of_code         ; end of all code (label)
                PUBLIC  _udelay              ; spin for microseconds
		EXTRN	DOSIODELAYCNT:ABS

; ----------------------------------------------------------------------------
; Device Driver Header

DEVHDR          SEGMENT WORD PUBLIC 'DATA'
_dev_hdr        dd      -1                      ; next device header
                dw      DEVLEV_3 + DEV_CHAR_DEV + DEV_IDC + DEV_30; flags for ADD drivers
                dw      OFFSET _asm_strat       ; strategy routine
                dw      OFFSET _asm_idc_entry   ; IDC entry point
                db      "OS2AHCI$"              ; name of character device
                dq      0                       ; 8 reserved bytes
                dd      DEV_IOCTL2 + DEV_ADAPTER_DD + DEV_INITCOMPLETE + DEV_SAVERESTORE ; ADD flags
                dw      0

DEVHDR          ENDS

; ----------------------------------------------------------------------------
; Segment Definitions. We need to reference all segments here, even if they
; are not used, to allow proper grouping of all segments into one group for
; data and one group for code as well as proper ordering of the segments
; within each group to make sure the end of segment markers are at the end
; of each group.

; On top of that, we need to make sure that the end of segment marker is in
; a data segment of class 'BSS' because BSS segments are always put towards
; the end of the executable when linking with high-level language objects
; such as C.

; When using the Watcom Linker, the segment ordering can be overridden in
; the Watcom Makefile (wmakefile) where the Linker response-file is generated
; containing statements for segment ordering. First you can order by class and
; for each class you can specify the order of the segments within that class.
; See the ORDER directive in the wlink documentation.
_DATA           SEGMENT WORD PUBLIC 'DATA'
readl_dbg_fmt   db      "readl(%04x:%04x) = 0x%08lx"
                db      10, 0
writel_dbg_fmt  db      "writel(%04x:%04x, 0x%08lx)"
                db      10, 0
_DATA           ENDS

LIBDATA         SEGMENT WORD PUBLIC 'DATA'
LIBDATA         ENDS

CONST           SEGMENT WORD PUBLIC 'CONST'
CONST           ENDS

_BSS            SEGMENT WORD PUBLIC 'BSS'
_BSS            ENDS

c_common        SEGMENT WORD PUBLIC 'BSS'
                EXTRN   _debug : WORD        ; global debug flag
c_common        ENDS

_z_data         SEGMENT WORD PUBLIC 'BSS'
_end_of_data    db      ?
_z_data         ENDS

_TEXT           SEGMENT WORD PUBLIC 'CODE'
                EXTRN   _c_strat : NEAR           ; C strategy routine
                EXTRN   _printf : NEAR            ; C printf routine
                EXTRN   _restart_ctxhook : NEAR   ; C restart context hook
                EXTRN   _reset_ctxhook : NEAR     ; C reset context hook
                EXTRN   _engine_ctxhook : NEAR    ; C engine context hook
                EXTRN   _shutdown_driver : NEAR   ; C routine for INT13 IO enable
_TEXT           ENDS

CODE            SEGMENT WORD PUBLIC 'CODE'
CODE            ENDS

LIBCODE         SEGMENT WORD PUBLIC 'CODE'
LIBCODE         ENDS

RMCode          SEGMENT WORD PUBLIC 'CODE'
RMCode          ENDS

_z_text         SEGMENT WORD PUBLIC 'CODE'
_end_of_code    LABEL NEAR
_z_text         ENDS


; The Watcom Linker behaves differently than the IBM/MS Linkers when segments
; are defined but not added to the corresponding group. So when you define a
; a new (logical) segment and want it to be part of a physical segment (group)
; then don't forget to add it below or unresolvable or miscalculated
; relocations can be the result.
DGROUP          GROUP   DEVHDR, _DATA, LIBDATA, _BSS, c_common, _z_data
TGROUP          GROUP   _TEXT, CODE, LIBCODE, RMCode, _z_text

        .386

; ----------------------------------------------------------------------------
; Start of code

_TEXT           SEGMENT WORD PUBLIC 'CODE'
                ASSUME  DS:DGROUP
                ASSUME  ES:NOTHING
                ASSUME  SS:NOTHING


; Device driver main entry point (strategy routine)
_asm_strat      PROC    FAR

                ; push request packet address
                PUSH    ES
                PUSH    BX
                CLD

                ; call C strategy routine
                CALL    _c_strat

                POP     BX
                POP     ES
                MOV     WORD PTR ES:[BX+3], AX
                RET
_asm_strat      ENDP


; IDC entry point (Assembler stub)
_asm_idc_entry  PROC    FAR

                ; push request packet address
                PUSH    ES
                PUSH    BX
                CLD

                ; call C IDC entry point - which is the strategy routine
                CALL    _c_strat

                POP     BX
                POP     ES
                MOV     WORD PTR ES:[BX+3], AX
                RET
_asm_idc_entry  ENDP

;  Kernel exit routine to enable INT13 I/O (called before a trap dump occurrs)
_asm_krnl_exit  PROC    FAR
                PUSH    ES
                PUSH    DS
                MOV     AX,DGROUP
                MOV     DS,AX
                CLD

                ;  call C routine
                CALL    _shutdown_driver

                POP     DS
                POP     ES

                RET
_asm_krnl_exit  ENDP


                .386

; void udelay(u16 microseconds);
_udelay         PROC NEAR
                enter   0, 0
                mov     cx, word ptr [bp+4]
@OuterLoop:
		mov	ax, DOSIODELAYCNT
		shl	ax, 1
@InnerLoop:
                dec     ax
                jnz     short @InnerLoop		
                loop    @OuterLoop

                leave
                ret
_udelay          ENDP

; Read long value from MMIO address; need to do this here to get real
; 32-bit operations because at least the AHCI device in VirtualBox doesn't
; seem to support reading 32-bit MMIO registers in two 16-bit steps.
;
; C prototype:  u32 readl(void _far *addr);
_readl          PROC    NEAR
                ENTER   0, 0

                ; load 'addr' into ES:EAX
                LES     AX, [BP+4]
                AND     EAX, 0000FFFFh

                ; read MMIO register into EDX and return it in DX:AX
                MOV     EDX, ES:[EAX]

                ; print debug message if debug level is 3+
                CMP     _debug, 3
                JB      no_debug
                PUSH    EDX             ; save value read from MMIO port
                PUSH    EDX             ; value read from MMIO address
                PUSH    AX              ; offset of MMIO address
                PUSH    ES              ; segment of MMIO address
                PUSH    OFFSET readl_dbg_fmt
                CALL    _printf
                ADD     SP, 10
                POP     EDX             ; restore value read from MMIO port

no_debug:       MOV     EAX, EDX
                SHR     EDX, 16

                LEAVE
                RET
_readl          ENDP


; Write long value to MMIO address; need to do this here to get real
; 32-bit operations because at least the AHCI device in VirtualBox doesn't
; seem to support reading 32-bit MMIO registers in two 16-bit steps and the
; assumption is that this is the same with writing 32-bit MMIO registers.
;
; C prototype:  void writel(void _far *addr, u32 val);
_writel         PROC    NEAR
                ENTER   0, 0

                ; load 'addr' into ES:EAX
                LES     AX, [BP+4]
                AND     EAX, 0000FFFFh

                ; load 'val' into EDX, preserving the upper 16 bits of EBP
                MOV     EBX, EBP
                AND     EBP, 0000FFFFh
                MOV     EDX, [EBP+8]
                MOV     EBP, EBX

                ; store 'val' at MMIO address in ES:EAX
                MOV     DWORD PTR ES:[EAX], EDX

                ; print debug message if debug level is 3+
                CMP     _debug, 3
                JB      no_debug2
                PUSH    EDX             ; value written to MMIO address
                PUSH    AX              ; offset of MMIO address
                PUSH    ES              ; segment of MMIO address
                PUSH    OFFSET writel_dbg_fmt
                CALL    _printf
                ADD     SP, 10

no_debug2:      LEAVE
                RET
_writel         ENDP


; Halfway-decent 32-bit implementation of memset().
;
; C prototype: void *memset(void _far *s, int c, size_t n);
_memset         PROC    NEAR
                ENTER   0, 0

                PUSH    DI

                ; load 's' into ES:EDI
                LES     DI, [BP+4]
                AND     EDI, 0000FFFFh

                ; load 'c' into EAX, replicating the low 8 bits all over
                MOV     BL, [BP+8]
                MOV     BH, BL
                MOV     AX, BX
                SHL     EAX, 16
                MOV     AX, BX

                ; load 'n / 4' into ECX
                MOV     CX, [BP+10]
                SHR     CX, 2
                AND     ECX, 0000FFFFh

                ; fill 's' with 32-bit chunks of EAX
                REP     STOSD

                ; set remaining bytes (n % 4)'
                MOV     CX, [BP+10]
                AND     CX, 0003h
                REP     STOSB

                ; return address of 's'
                LES     AX, [BP+4]
                PUSH    ES
                POP     DX

                POP     DI

                LEAVE
                RET
_memset         ENDP


; Halfway-decent 32-bit implementation of memcpy().
;
; C prototype: void *memcpy(void _far *d, void _far *s, size_t n);
_memcpy         PROC    NEAR
                ENTER   0, 0

                PUSH    SI
                PUSH    DI
                PUSH    DS

                ; load 'd' into ES:EDI
                LES     DI, [BP+4]
                AND     EDI, 0000FFFFh

                ; load 's' into DS:ESI
                LDS     SI, [BP+8]
                AND     ESI, 0000FFFFh

                ; load 'n / 4' into ECX
                MOV     CX, [BP+12]
                SHR     CX, 2
                AND     ECX, 0000FFFFh

                ; copy 's' to 'd' in 32-bit chunks
                REP     MOVSD

                ; copy remaining bytes (n % 4)'
                MOV     CX, [BP+12]
                AND     CX, 0003h
                REP     MOVSB

                ; return address of 'd'
                LES     AX, [BP+4]
                PUSH    ES
                POP     DX

                POP     DS
                POP     DI
                POP     SI

                LEAVE
                RET
_memcpy         ENDP


; Port restart context hook; context hooks need to save all registers
; and get the parameter passed to DevHelp_ArmCtxHook() in EAX, thus
; we need a stub which calls the real context hook.
_restart_hook   PROC    FAR
                PUSHAD
                PUSH    EAX
                CALL    _restart_ctxhook
                POP     EAX
                POPAD
                RET
_restart_hook   ENDP


; Port reset context hook; context hooks need to save all registers
; and get the parameter passed to DevHelp_ArmCtxHook() in EAX, thus
; we need a stub which calls the real context hook.
_reset_hook     PROC    FAR
                PUSHAD
                PUSH    EAX
                CALL    _reset_ctxhook
                POP     EAX
                POPAD
                RET
_reset_hook     ENDP


; Engine trigger context hook; context hooks need to save all registers
; and get the parameter passed to DevHelp_ArmCtxHook() in EAX, thus
; we need a stub which calls the real context hook.
_engine_hook    PROC    FAR
                PUSHAD
                PUSH    EAX
                CALL    _engine_ctxhook
                POP     EAX
                POPAD
                RET
_engine_hook    ENDP


; Unsigned long divide routine;
; taken from OS/2 Uniaud project, original author: Timur Tabi
__U4D           proc    near
                shl     edx,10h            ;; Load dx:ax into eax
                mov     dx,ax
                mov     eax,edx
                xor     edx,edx            ;; Zero extend eax into edx
                shl     ecx,10h            ;; Load cx:bx into ecx
                mov     cx,bx
                div     ecx                ;; Divide eax/ecx into eax
                mov     ecx,edx            ;; Load edx into cx:bx
                shr     ecx,10h
                mov     bx,dx
                mov     edx,eax            ;; Load eax into dx:ax
                shr     edx,10h
                ret
__U4D           endp

; Long multiply routine;
; taken from OS/2 Uniaud project, original author: Timur Tabi
__U4M           proc    near
__I4M           label   near
                shl     edx,10h            ;; Load dx:ax into eax
                mov     dx,ax
                mov     eax,edx
                mov     dx,cx              ;; Load cx:bx into edx
                shl     edx,10h
                mov     dx,bx
                mul     edx                ;; Multiply eax*edx into edx:eax
                mov     edx,eax            ;; Load eax into dx:ax
                shr     edx,10h
                ret
__U4M           endp


; Signed long divide routine;
; taken from OS/2 Uniaud project, original author: Timur Tabi
__I4D           proc    near
                shl     edx,10h            ;; Load dx:ax into eax
                mov     dx,ax
                mov     eax,edx
                cdq                        ;; Sign extend eax into edx
                shl     ecx,10h            ;; Load cx:bx into ecx
                mov     cx,bx
                idiv    ecx                ;; Divide eax/ecx into eax
                mov     ecx,edx            ;; Load edx into cx:bx
                shr     ecx,10h
                mov     bx,dx
                mov     edx,eax            ;; Load eax into dx:ax
                shr     edx,10h
                ret
__I4D           endp

_TEXT           ENDS

                END

