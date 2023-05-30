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
    
    ; TMR0 interruption
    BTFSC   INTCON, 2		; check T0IF bit
    CALL    TMR0ISR
    
    ; return previous context
    SWAPF   STATUS_TMP, W
    MOVWF   STATUS
    SWAPF   W_TMP, F
    SWAPF   W_TMP, W
    RETFIE

; program variables
W_TMP		EQU 0x20
STATUS_TMP	EQU 0x21
TMR0_CNTR	EQU 0x22

; LDRs
AN0_VALUE	EQU 0x23
AN1_VALUE	EQU 0x24
AN2_VALUE	EQU 0x25
AN3_VALUE	EQU 0x26
SNSBLTY		EQU 0x27

; stepper motor
STPR_MOTOR	EQU 0x28
TMR0_CNTR_REF	EQU 0x29

; program setup
setup:
    
    ; TRISA configuration
    BANKSEL TRISA
    MOVLW   0b00001111		; set <RA0:RA3> as inputs
    MOVWF   TRISA
    BANKSEL ANSEL
    MOVLW   0b00001111		; set <ANS0:ANS3> as analogs
    MOVWF   ANSEL
    
    ; TRISB configuration
    BANKSEL TRISB
    MOVLW   0b11111111		; set <RB0:RB7> as inputs
    MOVWF   TRISB
    BANKSEL ANSELH
    MOVLW   0b00000000		; set <ANS8:ANS13> as digitals
    MOVF    ANSELH
    
    ; PORTC configuration
    BANKSEL TRISC
    MOVLW   0b00000000		; set <RC0:RC7> PORTC pins as outputs to control LEDs and stepper motors
    MOVWF   TRISC
    
    ; general port configuration
    BANKSEL OPTION_REG		; enable global pull-ups and set pre-scaler (111) 
    MOVLW   0b00000111		; | /RBPU | INTEDG | T0CS | T0SE | PSA | PS2 | PS1 | PS0 |
    MOVWF   OPTION_REG
    BANKSEL WPUB
    MOVLW   0b11111111		; enable pull-ups in <RB0:RB7> pins
    MOVWF   WPUB

    ; interruption configuration
    BANKSEL INTCON		; enable global interruptions and interruptions in TMR0
    MOVLW   0b10100000		; | GIE | PEIE | T0IE | INTE | RBIE | T0IF | INTF | RBIF |
    MOVWF   INTCON
    BANKSEL IOCB
    MOVLW   0b00000000		; disable interruptions in <RB0:RB7> pins
    MOVWF   IOCB
    
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
    
    ; TMR0 initialization
    BANKSEL TMR0
    CLRF    TMR0
    
    ; PORTC initialization
    BANKSEL PORTC
    MOVLW   0b00000000
    MOVWF   PORTC
    
    ; sensibility initialization
    BANKSEL SNSBLTY
    MOVLW   0b11111000
    MOVWF   SNSBLTY

; main program loop
main:
    
    ; switch to channel AN0 and measure voltage on pin AN0
    BANKSEL ADCON0
    BCF	    ADCON0, 2		; set the ADC to measure voltage on pin AN0
    BCF	    ADCON0, 3
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete
    GOTO    $-1
    MOVF    ADRESH, W		; read the conversion result in ADRESH
    BANKSEL AN0_VALUE
    MOVWF   AN0_VALUE		; store the result in the variable AN0_VALUE

    ; switch to channel AN1 and measure voltage on pin AN1
    BANKSEL ADCON0
    BSF	    ADCON0, 2		; set the ADC to measure voltage on pin AN1
    BCF	    ADCON0, 3
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete (GO/DONE)
    GOTO    $-1
    MOVF    ADRESH, W		; read the conversion result in ADRESH
    BANKSEL AN1_VALUE
    MOVWF   AN1_VALUE		; store the result in the variable AN1_VALUE
    
    ; switch to channel AN2 and measure voltage on pin AN2
    BANKSEL ADCON0
    BCF	    ADCON0, 2		; set the ADC to measure voltage on pin AN2
    BSF	    ADCON0, 3
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete
    GOTO    $-1
    MOVF    ADRESH, W		; read the conversion result in ADRESH
    BANKSEL AN2_VALUE
    MOVWF   AN2_VALUE		; store the result in the variable AN2_VALUE
    
    ; switch to channel AN3 and measure voltage on pin AN3
    BANKSEL ADCON0
    BSF	    ADCON0, 2		; set the ADC to measure voltage on pin AN3
    BSF	    ADCON0, 3
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete
    GOTO    $-1
    MOVF    ADRESH, W		; read the conversion result in ADRESH
    BANKSEL AN3_VALUE
    MOVWF   AN3_VALUE		; store the result in the variable AN3_VALUE
    
    ; select memory bank 0 <00>
    BCF	    STATUS, 5		; clear RP0 bit
    BCF	    STATUS, 6		; clear RP1 bit
    
    ; move the light tracker
    CALL    moveUpDown
    CALL    moveLeftRight
    
    GOTO    main
    
; interruption subroutine to control TMR0
TMR0ISR:
    BANKSEL TMR0
    CLRF    TMR0		; reset TMR0
    INCF    TMR0_CNTR, F	; increment TMR0 counter variable
    BCF	    INTCON, 2		; clear T0IF bit
    RETURN
    
; subroutine to control the up/down stepper motor
moveUpDown:
    
    ; compare the measured voltages and rotate up or down if necessary
    MOVF    AN0_VALUE, W
    SUBWF   AN1_VALUE, W	; subtract AN1_VALUE from AN0_VALUE
    ANDWF   SNSBLTY, W		; apply sensitivity value to smooth motion
    BTFSC   STATUS, 2		; if the result is zero, do nothing
    GOTO    stopRotationUD
    BTFSS   STATUS, 0		; if the result is positive, rotate up
    GOTO    rotateUp
    BTFSC   STATUS, 0		; if the result is negative, rotate down
    GOTO    rotateDown
    
    ; rotate one step up
    rotateUp:
	BSF	PORTC, 0	; set direction (RC0) in HIGH
	BSF	PORTC, 1	; set pulse (RC1) in HIGH
	CALL	getDelay
	BCF	PORTC, 1	; set pulse (RC1) in LOW
	CALL	getDelay
	RETURN
    
    ; rotate one step down
    rotateDown:
	BCF	PORTC, 0	; set direction (RC0) in LOW
	BSF	PORTC, 1	; set pulse (RC1) in HIGH
	CALL	getDelay
	BCF	PORTC, 1	; set pulse (RC1) in LOW
	CALL	getDelay
	RETURN
    
    ; no rotation
    stopRotationUD:
	BCF	PORTC, 0	; set direction (RC0) in LOW
	BCF	PORTC, 1	; set pulse (RC1) in LOW
	CALL	getDelay
	CALL	getDelay	; make two delays to complete one cycle
	RETURN
	
; subroutine to control the up/down stepper motor
moveLeftRight:
    
    ; compare the measured voltages and rotate left or right if necessary
    MOVF    AN2_VALUE, W
    SUBWF   AN3_VALUE, W	; subtract AN3_VALUE from AN2_VALUE
    ANDWF   SNSBLTY, W		; apply sensitivity value to smooth motion
    BTFSC   STATUS, 2		; if the result is zero, do nothing
    GOTO    stopRotationLR
    BTFSS   STATUS, 0		; if the result is positive, rotate left
    GOTO    rotateLeft
    BTFSC   STATUS, 0		; if the result is negative, rotate right
    GOTO    rotateRight
    
    ; rotate one step up
    rotateLeft:
	BSF	PORTC, 2	; set direction (RC2) in HIGH
	BSF	PORTC, 3	; set pulse (RC3) in HIGH
	CALL	getDelay
	BCF	PORTC, 3	; set pulse (RC3) in LOW
	CALL	getDelay
	RETURN
    
    ; rotate one step down
    rotateRight:
	BCF	PORTC, 2	; set direction (RC2) in LOW
	BSF	PORTC, 3	; set pulse (RC3) in HIGH
	CALL	getDelay
	BCF	PORTC, 3	; set pulse (RC3) in LOW
	CALL	getDelay
	RETURN
    
    ; no rotation
    stopRotationLR:
	BCF	PORTC, 2	; set direction (RC2) in LOW
	BCF	PORTC, 3	; set pulse (RC3) in LOW
	CALL	getDelay
	CALL	getDelay	; make two delays to complete one cycle
	RETURN

; subroutine to make a delay
getDelay:
    
    ; set the reference value of TMR0
    MOVF    TMR0_CNTR, W
    MOVWF   TMR0_CNTR_REF
    
    ; compare the actual TMR0 counter with the reference to make a pulse
    MOVF    TMR0_CNTR, W
    SUBWF   TMR0_CNTR_REF, W
    BTFSC   STATUS, 2
    GOTO    $-3
    RETURN
    
END RESET_VECT