#include "xc.inc"

; CONFIG1
  CONFIG  FOSC = XT             ; Oscillator Selection bits (XT oscillator: Crystal/resonator on RA6/OSC2/CLKOUT and RA7/OSC1/CLKIN)
  CONFIG  WDTE = OFF            ; Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
  CONFIG  PWRTE = OFF           ; Power-up Timer Enable bit (PWRT disabled)
  CONFIG  MCLRE = ON            ; RE3/MCLR pin function select bit (RE3/MCLR pin function is MCLR)
  CONFIG  CP = OFF              ; Code Protection bit (Program memory code protection is disabled)
  CONFIG  CPD = OFF             ; Data Code Protection bit (Data memory code protection is disabled)
  CONFIG  BOREN = OFF           ; Brown Out Reset Selection bits (BOR disabled)
  CONFIG  IESO = OFF            ; Internal External Switchover bit (Internal/External Switchover mode is disabled)
  CONFIG  FCMEN = OFF           ; Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
  CONFIG  LVP = OFF             ; Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

; CONFIG2
  CONFIG  BOR4V = BOR40V        ; Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
  CONFIG  WRT = OFF             ; Flash Program Memory Self Write Enable bits (Write protection off)

; starting position of the program < -pRESET_VECT=0h >
psect RESET_VECT, class=CODE, delta=2
RESET_VECT:
    GOTO    setup

; memory location to go when a interrupt happens < -pINT_VECT=4h >
psect INT_VECT, class=CODE, delta=2
INT_VECT:
    
    ; save context
    MOVWF   W_TMP
    SWAPF   STATUS, W
    MOVWF   STATUS_TMP
    
    ; IMPLEMENT METHOD INTERRUPTION
    
    ; return previous context
    SWAPF   STATUS_TMP, W
    MOVWF   STATUS
    SWAPF   W_TMP, F
    SWAPF   W_TMP, W
    RETFIE

; program variables
W_TMP		EQU 0x20
STATUS_TMP	EQU 0x21

; LDRs
AN0_VALUE	EQU 0x22
AN1_VALUE	EQU 0x23
LDR_SNSBLTY_0	EQU 0x24

; TMR0 counter
STP_DLAY_CTER	EQU 0X25

; program setup
setup:
    
    ; set LDR sensibility
    MOVLW   0b11110000		; set the sensibility value to delete possible noise
    MOVWF   LDR_SNSBLTY_0
    
    ; ports configuration
    BANKSEL TRISA
    MOVLW   0b00000011		; set <AN0:AN1> as inputs
    MOVWF   TRISA
    BANKSEL TRISB
    MOVLW   0b00000000		; set <RB0:RB3> as outputs
    MOVWF   TRISB
    BANKSEL ANSEL
    MOVLW   0b00000011		; enable analog inputs on <AN0:AN1>
    MOVWF   ANSEL

    ; ADC configuration
    BANKSEL VRCON		; set the reference voltage
    MOVLW   0b00000000		; | VREN | VROE | VRR | VRSS | VR3 | VR2 | VR1 | VR0 |
    MOVWF   VRCON
    BANKSEL ADCON0		; set the max clock, set the input channel AN0 and turn on the ADC
    MOVLW   0b10000001		; | ADCS1 | ADCS0 | CHS3 | CHS2 | CHS1 | CHS0 | GO/DONE | ADON |
    MOVWF   ADCON0
    BANKSEL ADCON1		; set reference voltage source in VDD & VSS ans justify the result to the left
    MOVLW   0b00000000		; | ADFM | xx | VCFG1 | VCFG0 | xx | xx | xx | xx |
    MOVWF   ADCON1

; main program loop
main:
    
    ; switch to channel AN0 and measure voltage on pin AN0
    BANKSEL ADCON0
    BCF	    ADCON0, 2		; set the ADC to measure voltage on pin AN0
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete
    GOTO    $-1
    MOVF    ADRESH, W		; read the conversion result in ADRESH
    MOVWF   AN0_VALUE		; store the result in the variable AN0_VALUE

    ; switch to channel AN1 and measure voltage on pin AN1
    BANKSEL ADCON0
    BSF	    ADCON0, 2		; set the ADC to measure voltage on pin AN1
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete (GO/DONE)
    GOTO    $-1
    MOVF    ADRESH, W		; read the conversion result in ADRESH
    MOVWF   AN1_VALUE		; store the result in the variable AN1_VALUE

    ; apply sensibility value to the measured voltages to delete possible noise
    MOVF    AN0_VALUE, W
    ANDLW   LDR_SNSBLTY_0
    MOVWF   AN0_VALUE
    MOVF    AN1_VALUE, W
    ANDLW   LDR_SNSBLTY_0
    MOVWF   AN1_VALUE
    
    ; compare the measured voltages and rotate to left or right if necessary
    MOVF    AN0_VALUE, W
    SUBWF   AN1_VALUE, W	; subtract AN1_VALUE from AN0_VALUE
    BTFSC   STATUS, 2		; if the result is zero, do nothing and skip
    GOTO    $+5
    BTFSS   STATUS, 0		; if the result is positive, rotate to the left
    CALL    rotateLeft
    BTFSC   STATUS, 0		; if the result is negative, rotate to the right
    CALL    rotateRight
    
    GOTO    main
    
; subroutine to rotate to the left
rotateLeft:
    MOVLW   0b00001000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00000100
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00000010
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00000001
    MOVWF   PORTB
    CALL    stpDelay
    
    ; turn off PORTB
    MOVLW   0b00000000
    MOVWF   PORTB
    RETURN
    
; subroutine to rotate to the right
rotateRight:
    MOVLW   0b00001000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00000001
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00000010
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00000100
    MOVWF   PORTB
    CALL    stpDelay
    
    ; turn off PORTB
    MOVLW   0b00000000
    MOVWF   PORTB
    RETURN
    
; subroutine to rotate up
rotateUp:
    MOVLW   0b10000000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b01000000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00100000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00010000
    MOVWF   PORTB
    CALL    stpDelay
    
    ; turn off PORTB
    MOVLW   0b00000000
    MOVWF   PORTB
    RETURN
    
; subroutine to rotate down
rotateDown:
    MOVLW   0b10000000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00010000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b00100000
    MOVWF   PORTB
    CALL    stpDelay
    MOVLW   0b01000000
    MOVWF   PORTB
    CALL    stpDelay
    
    ; turn off PORTB
    MOVLW   0b00000000
    MOVWF   PORTB
    RETURN
    
; steps delay subroutine (using instructions)
stpDelay:
    MOVLW   0x2F		; initial value of mayor loop
    MOVWF   STP_DLAY_CTER_0
stpLoop_1:
    MOVLW   0xFF		; initial value of minor loop
    MOVWF   STP_DLAY_CTER_1
stpLoop_0:
    DECFSZ  STP_DLAY_CTER_1
    GOTO    stpLoop_0
    DECFSZ  STP_DLAY_CTER_0
    GOTO    stpLoop_1
    RETURN

END RESET_VECT