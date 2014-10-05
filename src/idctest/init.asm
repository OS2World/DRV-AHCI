; ----------------------------------------------------------------------------
; Initialization routines and segment mappings for idctest driver.
;
; This is a stripped down version of os2ahci's init.asm.

include devhdr.inc

; -----------------------------------------------------------------------------
; Public symbols

                PUBLIC  _asm_strat           ; low-level strategy routine
                PUBLIC  _end_of_data         ; end of all data (label)
                PUBLIC  _end_of_code         ; end of all code (label)
        
; ----------------------------------------------------------------------------
; Device Driver Header

DEVHDR          SEGMENT WORD PUBLIC 'DATA'
_dev_hdr        dd      -1                      ; next device header
                dw      DEVLEV_3 + DEV_CHAR_DEV ; flags for ADD drivers
                dw      OFFSET _asm_strat       ; strategy routine
                dw      0                       ; IDC entry point
                db      "IDCTEST$"              ; name of character device
                dq      0                       ; 8 reserved bytes
                dd      0                       ; ADD flags
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

_DATA           SEGMENT WORD PUBLIC 'DATA'
_DATA           ENDS

CONST           SEGMENT WORD PUBLIC 'CONST'
CONST           ENDS

_BSS            SEGMENT WORD PUBLIC 'BSS'
_BSS            ENDS

c_common        SEGMENT WORD PUBLIC 'BSS'
c_common        ENDS

_z_data         SEGMENT WORD PUBLIC 'BSS'
_end_of_data    db      0
_z_data         ENDS

_TEXT           SEGMENT WORD PUBLIC 'CODE'
                EXTRN   _c_strat_idc : NEAR           ; C strategy routine
_TEXT           ENDS

CODE            SEGMENT WORD PUBLIC 'CODE'
CODE            ENDS

RMCode          SEGMENT WORD PUBLIC 'CODE'
RMCode          ENDS

LIBCODE         SEGMENT WORD PUBLIC 'CODE'
LIBCODE         ENDS

_z_text         SEGMENT WORD PUBLIC 'CODE'
_end_of_code    LABEL NEAR
_z_text         ENDS

DGROUP          GROUP   DEVHDR, _DATA, CONST, _BSS, c_common, _z_data
TGROUP          GROUP   _TEXT, CODE, _z_text

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
                CALL    _c_strat_idc

                POP     BX
                POP     ES
                MOV     WORD PTR ES:[BX+3], AX
                RET
_asm_strat      ENDP


_TEXT           ENDS

                END

