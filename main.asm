
; ATmega64 - Traffic Light Controller (FSM + Timer + INT0/1/2 + Traffic Sensor)
; Tick = 10ms using Timer1 CTC
;
; Outputs (PORTA and duplicated on PORTB bits 0..5):
;  bit0 NS_R, bit1 NS_Y, bit2 NS_G, bit3 EW_R, bit4 EW_Y, bit5 EW_G
;
; Buttons:
;  INT0: PD2 (falling)  short/long 1.5s
;  INT1: PD3 (falling)  short/long 1.5s
;  INT2: PB2 (falling)  short/long 1.0s  (yellow adjust 250ms)
;
; Traffic sensor:
;  PC1..PC0 : 00 normal, 01 NS traffic, 10 EW traffic


.include "m64def.inc"


; Clock / Timer constants

.equ OCR1A_10MS = 1249          ; 8MHz, presc=64 => 10ms
; .equ OCR1A_10MS = 2499        ; 16MHz, presc=64 => 10ms

.equ TICKS_1S   = 100
.equ TICKS_15S  = 150
.equ TICKS_250M = 25

; Limits
.equ G_MIN_S = 5
.equ G_MAX_S = 30
.equ Y_MIN_T = 200              ; 2.0s
.equ Y_MAX_T = 300              ; 3.0s

; Defaults
.equ DEF_G_NS_S = 5
.equ DEF_G_EW_S = 5
.equ DEF_Y_NS_T = 50
.equ DEF_Y_EW_T = 50

; FSM states
.equ ST_NS_GREEN  = 0
.equ ST_NS_YELLOW = 1
.equ ST_EW_GREEN  = 2
.equ ST_EW_YELLOW = 3

; Pending actions
.equ ACT_NONE = 0
.equ ACT_INT0_SHORT = 1
.equ ACT_INT0_LONG  = 2
.equ ACT_INT1_SHORT = 3
.equ ACT_INT1_LONG  = 4
.equ ACT_INT2_SHORT = 5
.equ ACT_INT2_LONG  = 6

; Output masks (PORTA/B)
.equ OUT_NS_G = (1<<2) | (1<<3)
.equ OUT_NS_Y = (1<<1) | (1<<3)
.equ OUT_EW_G = (1<<5) | (1<<0)
.equ OUT_EW_Y = (1<<4) | (1<<0)


; SRAM

.dseg
state:          .byte 1
phaseL:         .byte 1
phaseH:         .byte 1
phase_done:     .byte 1

g_ns_s:         .byte 1
g_ew_s:         .byte 1
y_nsL:          .byte 1
y_nsH:          .byte 1
y_ewL:          .byte 1
y_ewH:          .byte 1

traffic_mode:   .byte 1
save_g_ns:      .byte 1
save_g_ew:      .byte 1
save_y_nsL:     .byte 1
save_y_nsH:     .byte 1
save_y_ewL:     .byte 1
save_y_ewH:     .byte 1

btn0_active:    .byte 1
btn1_active:    .byte 1
btn2_active:    .byte 1
btn0L:          .byte 1
btn0H:          .byte 1
btn1L:          .byte 1
btn1H:          .byte 1
btn2L:          .byte 1
btn2H:          .byte 1

pending_act:    .byte 1

.cseg
.org 0x0000
    jmp RESET

.org INT0addr
    jmp ISR_INT0
.org INT1addr
    jmp ISR_INT1
.org INT2addr
    jmp ISR_INT2
.org OC1Aaddr
    jmp ISR_T1_COMPA


; RESET

RESET:
    ; Stack
    ldi r16, high(RAMEND)
    out SPH, r16
    ldi r16, low(RAMEND)
    out SPL, r16

    ; Outputs: PORTA/B bits 0..5
    ldi r16, 0x3F
    out DDRA, r16
    out DDRB, r16

    ; Sensor inputs PC0, PC1
    cbi DDRC, 0
    cbi DDRC, 1

    ; INT0 PD2, INT1 PD3 input + pull-up
    cbi DDRD, 2
    cbi DDRD, 3
    sbi PORTD, 2
    sbi PORTD, 3

    ; INT2 PB2 input + pull-up
    cbi DDRB, 2
    sbi PORTB, 2

    ; Init vars
    ldi r16, ST_NS_GREEN
    sts state, r16

    clr r16
    sts phase_done, r16
    sts pending_act, r16
    sts traffic_mode, r16

    sts btn0_active, r16
    sts btn1_active, r16
    sts btn2_active, r16
    sts btn0L, r16
    sts btn0H, r16
    sts btn1L, r16
    sts btn1H, r16
    sts btn2L, r16
    sts btn2H, r16

    ldi r16, DEF_G_NS_S
    sts g_ns_s, r16
    ldi r16, DEF_G_EW_S
    sts g_ew_s, r16

    ldi r16, low(DEF_Y_NS_T)
    sts y_nsL, r16
    ldi r16, high(DEF_Y_NS_T)
    sts y_nsH, r16

    ldi r16, low(DEF_Y_EW_T)
    sts y_ewL, r16
    ldi r16, high(DEF_Y_EW_T)
    sts y_ewH, r16

    
    ; Timer1 CTC OCR1A 10ms
    
    clr r16
    out TCCR1A, r16
    ldi r16, (1<<WGM12)
    out TCCR1B, r16

    ldi r16, low(OCR1A_10MS)
    out OCR1AL, r16
    ldi r16, high(OCR1A_10MS)
    out OCR1AH, r16

    ; Enable OC1A interrupt
    ldi r16, (1<<OCIE1A)
    out TIMSK, r16

    ; Start Timer1 presc=64
    in  r16, TCCR1B
    ori r16, (1<<CS11) | (1<<CS10)
    out TCCR1B, r16

    
    ; External interrupts
    
    ldi r16, (1<<ISC01) | (1<<ISC11)   ; INT0/INT1 falling
    out MCUCR, r16

    in  r16, MCUCSR
    ori r16, (1<<6)                    ; ISC2 bit6 (INT2 falling)
    out MCUCSR, r16

    ldi r16, (1<<INT0) | (1<<INT1) | (1<<INT2)
    out GICR, r16

    sei

    rcall APPLY_OUTPUTS
    rcall LOAD_PHASE

MAIN_LOOP:
    ; phase done?
    lds r16, phase_done
    tst r16
    breq CHECK_PENDING

    clr r16
    sts phase_done, r16

    ; next state
    lds r16, state
    inc r16
    cpi r16, 4
    brlo STATE_OK
    clr r16
STATE_OK:
    sts state, r16

    ; Traffic check when entering GREEN
    cpi r16, ST_NS_GREEN
    breq DO_TR
    cpi r16, ST_EW_GREEN
    breq DO_TR
    jmp AFTER_TR
DO_TR:
    rcall TRAFFIC_MANAGER
AFTER_TR:
    rcall APPLY_OUTPUTS
    rcall LOAD_PHASE

CHECK_PENDING:
    lds r16, pending_act
    cpi r16, ACT_NONE
    breq MAIN_LOOP

    ; ignore manual changes in traffic forced mode
    lds r17, traffic_mode
    tst r17
    brne CLEAR_PENDING

    cpi r16, ACT_INT0_SHORT
    breq ACT_A0S
    cpi r16, ACT_INT0_LONG
    breq ACT_A0L
    cpi r16, ACT_INT1_SHORT
    breq ACT_A0S
    cpi r16, ACT_INT1_LONG
    breq ACT_A0L
    cpi r16, ACT_INT2_SHORT
    breq ACT_A2S
    cpi r16, ACT_INT2_LONG
    breq ACT_A2L
    jmp CLEAR_PENDING

ACT_A0S:
    rcall INC_NS_DEC_EW_GREEN
    jmp CLEAR_PENDING
ACT_A0L:
    rcall DEC_NS_INC_EW_GREEN
    jmp CLEAR_PENDING
ACT_A2S:
    rcall INC_YNS_DEC_YEW
    jmp CLEAR_PENDING
ACT_A2L:
    rcall INC_YEW_DEC_YNS
    jmp CLEAR_PENDING

CLEAR_PENDING:
    ldi r16, ACT_NONE
    sts pending_act, r16
    jmp MAIN_LOOP


; INT0/1/2: start measuring

ISR_INT0:
    push r16
    ldi r16, 1
    sts btn0_active, r16
    clr r16
    sts btn0L, r16
    sts btn0H, r16
    pop r16
    reti

ISR_INT1:
    push r16
    ldi r16, 1
    sts btn1_active, r16
    clr r16
    sts btn1L, r16
    sts btn1H, r16
    pop r16
    reti

ISR_INT2:
    push r16
    ldi r16, 1
    sts btn2_active, r16
    clr r16
    sts btn2L, r16
    sts btn2H, r16
    pop r16
    reti


; Timer tick ISR (10ms)

ISR_T1_COMPA:
    push r16
    push r17
    push r18
    push r19
    push r24
    push r25

    ; phase-- (r25:r24)
    lds r24, phaseL
    lds r25, phaseH
    subi r24, 1
    sbci r25, 0
    sts phaseL, r24
    sts phaseH, r25
    brne PH_OK
    ldi r16, 1
    sts phase_done, r16
PH_OK:

    ; BTN0 PD2
    lds r16, btn0_active
    tst r16
    breq BTN1_CHECK
    in  r19, PIND
    sbrc r19, 2
    jmp BTN0_RELEASE
    lds r24, btn0L
    lds r25, btn0H
    adiw r24, 1
    sts btn0L, r24
    sts btn0H, r25
    jmp BTN1_CHECK

BTN0_RELEASE:
    clr r16
    sts btn0_active, r16
    lds r24, btn0L
    lds r25, btn0H
    cpi r24, low(TICKS_15S)
    ldi r18, high(TICKS_15S)
    cpc r25, r18
    brsh BTN0_LONG
    ldi r16, ACT_INT0_SHORT
    sts pending_act, r16
    jmp BTN1_CHECK
BTN0_LONG:
    ldi r16, ACT_INT0_LONG
    sts pending_act, r16

BTN1_CHECK:
    ; BTN1 PD3
    lds r16, btn1_active
    tst r16
    breq BTN2_CHECK
    in  r19, PIND
    sbrc r19, 3
    jmp BTN1_RELEASE
    lds r24, btn1L
    lds r25, btn1H
    adiw r24, 1
    sts btn1L, r24
    sts btn1H, r25
    jmp BTN2_CHECK

BTN1_RELEASE:
    clr r16
    sts btn1_active, r16
    lds r24, btn1L
    lds r25, btn1H
    cpi r24, low(TICKS_15S)
    ldi r18, high(TICKS_15S)
    cpc r25, r18
    brsh BTN1_LONG
    ldi r16, ACT_INT1_SHORT
    sts pending_act, r16
    jmp BTN2_CHECK
BTN1_LONG:
    ldi r16, ACT_INT1_LONG
    sts pending_act, r16

BTN2_CHECK:
    ; BTN2 PB2
    lds r16, btn2_active
    tst r16
    breq ISR_DONE
    in  r19, PINB
    sbrc r19, 2
    jmp BTN2_RELEASE
    lds r24, btn2L
    lds r25, btn2H
    adiw r24, 1
    sts btn2L, r24
    sts btn2H, r25
    jmp ISR_DONE

BTN2_RELEASE:
    clr r16
    sts btn2_active, r16
    lds r24, btn2L
    lds r25, btn2H
    cpi r24, low(TICKS_1S)
    ldi r18, high(TICKS_1S)
    cpc r25, r18
    brsh BTN2_LONG
    ldi r16, ACT_INT2_SHORT
    sts pending_act, r16
    jmp ISR_DONE
BTN2_LONG:
    ldi r16, ACT_INT2_LONG
    sts pending_act, r16

ISR_DONE:
    pop r25
    pop r24
    pop r19
    pop r18
    pop r17
    pop r16
    reti


; Outputs

APPLY_OUTPUTS:
    push r16
    lds r16, state
    cpi r16, ST_NS_GREEN
    breq OUT0
    cpi r16, ST_NS_YELLOW
    breq OUT1
    cpi r16, ST_EW_GREEN
    breq OUT2
    ldi r16, OUT_EW_Y
    jmp OUTX
OUT0:
    ldi r16, OUT_NS_G
    jmp OUTX
OUT1:
    ldi r16, OUT_NS_Y
    jmp OUTX
OUT2:
    ldi r16, OUT_EW_G
OUTX:
    out PORTA, r16
    out PORTB, r16
    pop r16
    ret


; Load phase counter

LOAD_PHASE:
    push r16
    push r18
    push r24
    push r25

    lds r16, state
    cpi r16, ST_NS_GREEN
    breq LP0
    cpi r16, ST_NS_YELLOW
    breq LP1
    cpi r16, ST_EW_GREEN
    breq LP2
    jmp LP3

LP0:
    lds r18, g_ns_s
    rcall SEC100
    jmp LP_STORE
LP2:
    lds r18, g_ew_s
    rcall SEC100
    jmp LP_STORE
LP1:
    lds r24, y_nsL
    lds r25, y_nsH
    jmp LP_STORE2
LP3:
    lds r24, y_ewL
    lds r25, y_ewH

LP_STORE2:
    sts phaseL, r24
    sts phaseH, r25
    jmp LP_DONE

LP_STORE:
    sts phaseL, r24
    sts phaseH, r25

LP_DONE:
    pop r25
    pop r24
    pop r18
    pop r16
    ret

SEC100:
    clr r24
    clr r25
SEC100_LOOP:
    tst r18
    breq SEC100_DONE
    ldi r16, 100
    add r24, r16
    clr r16
    adc r25, r16
    dec r18
    jmp SEC100_LOOP
SEC100_DONE:
    ret


; Manual tuning

INC_NS_DEC_EW_GREEN:
    push r16
    push r17
    lds r16, g_ns_s
    lds r17, g_ew_s
    cpi r16, G_MAX_S
    brsh ID_FAIL
    cpi r17, G_MIN_S
    breq ID_FAIL
    inc r16
    dec r17
    sts g_ns_s, r16
    sts g_ew_s, r17
ID_FAIL:
    pop r17
    pop r16
    ret

DEC_NS_INC_EW_GREEN:
    push r16
    push r17
    lds r16, g_ns_s
    lds r17, g_ew_s
    cpi r16, G_MIN_S
    breq DI_FAIL
    cpi r17, G_MAX_S
    brsh DI_FAIL
    dec r16
    inc r17
    sts g_ns_s, r16
    sts g_ew_s, r17
DI_FAIL:
    pop r17
    pop r16
    ret

INC_YNS_DEC_YEW:
    push r24
    push r25
    lds r24, y_nsL
    lds r25, y_nsH
    rcall ADD25_Y
    sts y_nsL, r24
    sts y_nsH, r25
    lds r24, y_ewL
    lds r25, y_ewH
    rcall SUB25_Y
    sts y_ewL, r24
    sts y_ewH, r25
    pop r25
    pop r24
    ret

INC_YEW_DEC_YNS:
    push r24
    push r25
    lds r24, y_ewL
    lds r25, y_ewH
    rcall ADD25_Y
    sts y_ewL, r24
    sts y_ewH, r25
    lds r24, y_nsL
    lds r25, y_nsH
    rcall SUB25_Y
    sts y_nsL, r24
    sts y_nsH, r25
    pop r25
    pop r24
    ret

ADD25_Y:
    cpi r24, low(Y_MAX_T)
    ldi r16, high(Y_MAX_T)
    cpc r25, r16
    brsh ADD25_DONE
    ldi r16, TICKS_250M
    add r24, r16
    clr r16
    adc r25, r16
    cpi r24, low(Y_MAX_T)
    ldi r16, high(Y_MAX_T)
    cpc r25, r16
    brlo ADD25_DONE
    ldi r24, low(Y_MAX_T)
    ldi r25, high(Y_MAX_T)
ADD25_DONE:
    ret

SUB25_Y:
    cpi r24, low(Y_MIN_T)
    ldi r16, high(Y_MIN_T)
    cpc r25, r16
    brlo SUB25_DONE
    breq SUB25_DONE
    ldi r16, TICKS_250M
    sub r24, r16
    clr r16
    sbc r25, r16
    cpi r24, low(Y_MIN_T)
    ldi r16, high(Y_MIN_T)
    cpc r25, r16
    brsh SUB25_DONE
    ldi r24, low(Y_MIN_T)
    ldi r25, high(Y_MIN_T)
SUB25_DONE:
    ret


; TRAFFIC MANAGER (NO FAR RELATIVE BRANCHES)

TRAFFIC_MANAGER:
    push r16
    push r17

    in  r16, PINC
    andi r16, 0x03

    tst r16
    breq TM_NORMAL_REL
    jmp TM_NOT_ZERO
TM_NORMAL_REL:
    jmp TM_NORMAL

TM_NOT_ZERO:
    ; if already traffic -> done
    lds r17, traffic_mode
    tst r17
    breq TM_ENTER_REL
    jmp TM_DONE
TM_ENTER_REL:
    jmp TM_ENTER

TM_ENTER:
    ; save current
    ldi r17, 1
    sts traffic_mode, r17

    lds r17, g_ns_s
    sts save_g_ns, r17
    lds r17, g_ew_s
    sts save_g_ew, r17
    lds r17, y_nsL
    sts save_y_nsL, r17
    lds r17, y_nsH
    sts save_y_nsH, r17
    lds r17, y_ewL
    sts save_y_ewL, r17
    lds r17, y_ewH
    sts save_y_ewH, r17

    ; decide 01 / 10 using near-branch-to-jmp pattern
    cpi r16, 0x01
    brne TM_SKIP_NS
    jmp FORCE_NS
TM_SKIP_NS:
    cpi r16, 0x02
    brne TM_SKIP_EW
    jmp FORCE_EW
TM_SKIP_EW:
    jmp TM_DONE

FORCE_NS:
    ldi r17, G_MAX_S
    sts g_ns_s, r17
    ldi r17, G_MIN_S
    sts g_ew_s, r17
    ldi r17, low(Y_MIN_T)
    sts y_nsL, r17
    sts y_ewL, r17
    ldi r17, high(Y_MIN_T)
    sts y_nsH, r17
    sts y_ewH, r17
    jmp TM_DONE

FORCE_EW:
    ldi r17, G_MIN_S
    sts g_ns_s, r17
    ldi r17, G_MAX_S
    sts g_ew_s, r17
    ldi r17, low(Y_MIN_T)
    sts y_nsL, r17
    sts y_ewL, r17
    ldi r17, high(Y_MIN_T)
    sts y_nsH, r17
    sts y_ewH, r17
    jmp TM_DONE

TM_NORMAL:
    ; if traffic_mode=1 => restore else done
    lds r17, traffic_mode
    tst r17
    breq TM_DONE_REL
    jmp TM_RESTORE
TM_DONE_REL:
    jmp TM_DONE

TM_RESTORE:
    clr r17
    sts traffic_mode, r17

    lds r17, save_g_ns
    sts g_ns_s, r17
    lds r17, save_g_ew
    sts g_ew_s, r17
    lds r17, save_y_nsL
    sts y_nsL, r17
    lds r17, save_y_nsH
    sts y_nsH, r17
    lds r17, save_y_ewL
    sts y_ewL, r17
    lds r17, save_y_ewH
    sts y_ewH, r17

TM_DONE:
    pop r17
    pop r16
    ret
