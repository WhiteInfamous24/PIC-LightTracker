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
    
    ; keyboard/limit switchs interruption
    BANKSEL INTCON
    BTFSC   INTCON, 0		; check RBIF bit
    CALL    keyboardISR
    
    ; TMR0 interruption
    BANKSEL INTCON
    BTFSC   INTCON, 2		; check T0IF bit
    CALL    TMR0ISR
    
    ; ADC interruption
    BANKSEL PIR1
    BTFSC   PIR1, 6		; check ADIF bit
    CALL    ADCISR
    
    ; return previous context
    SWAPF   STATUS_TMP, W
    MOVWF   STATUS
    SWAPF   W_TMP, F
    SWAPF   W_TMP, W
    RETFIE

; interruptions context variables
W_TMP		EQU 0x20	; temporary W
STATUS_TMP	EQU 0x21	; temporary STATUS

; timer 0
TMR0_CNTR	EQU 0x30	; TMR0 counter
TMR0_CNTR_REF	EQU 0x31	; TMR0 counter temporary reference

; ADC
AN0_VALUE	EQU 0x40
AN1_VALUE	EQU 0x41
AN2_VALUE	EQU 0x42
AN3_VALUE	EQU 0x43
ADC_PORT_IT	EQU 0x44	; ADC port iterator
SNSBLTY_RANGE	EQU 0x45	; sensibility range to prevent oscilations

; keyboard
KYBRD_BTN	EQU 0x50
KYBRD_FND_F	EQU 0x51

; program setup
setup:
    
    ; PORTA configuration (LDRs)
    BANKSEL TRISA
    MOVLW   0b00001111		; set <RA0:RA3> as inputs
    MOVWF   TRISA
    BANKSEL ANSEL
    MOVLW   0b00001111		; set <ANS0:ANS3> as analogs
    MOVWF   ANSEL
    
    ; PORTB configuration (keyboard rows/limit switchs)
    BANKSEL TRISB
    MOVLW   0b11111111		; set <RB0:RB7> as inputs
    MOVWF   TRISB
    BANKSEL ANSELH
    MOVLW   0b00000000		; set <ANS8:ANS13> as digitals
    MOVF    ANSELH
    
    ; PORTC configuration (LEDs & stepper motors)
    BANKSEL TRISC
    MOVLW   0b00000000		; set <RC0:RC7> as outputs
    MOVWF   TRISC
    
    ; PORTD configuration (keyboard columns)
    BANKSEL TRISD
    MOVLW   0b00000000		; set <RD4:RD7> as outputs 
    MOVWF   TRISD
    
    ; general port configuration
    BANKSEL OPTION_REG		; enable global pull-ups and set pre-scaler (100=fast, 110=slow) 
    MOVLW   0b00000100		; | /RBPU | INTEDG | T0CS | T0SE | PSA | PS2 | PS1 | PS0 |
    MOVWF   OPTION_REG
    BANKSEL WPUB
    MOVLW   0b11111111		; enable pull-ups in <RB0:RB7>
    MOVWF   WPUB

    ; interruptions configuration
    BANKSEL INTCON		; enable global interruptions, interruptions in PEIE, interruptions in TMR0 and interruptions in PORTC
    MOVLW   0b11111000		; | GIE | PEIE | T0IE | INTE | RBIE | T0IF | INTF | RBIF |
    MOVWF   INTCON
    BANKSEL IOCB
    MOVLW   0b11111111		; enable interruptions in <RB0:RB7>
    MOVWF   IOCB
    BANKSEL PIE1		; enable interruptions in ADC
    MOVLW   0b01000000		; | xx | ADIE | RCIE | TXIE | SSPIE | CCP1IE | TMR2IE | TMR1IE |
    MOVWF   PIE1
    
    ; ADC configuration
    BANKSEL VRCON		; set the reference voltage
    MOVLW   0b00000000		; | VREN | VROE | VRR | VRSS | VR3 | VR2 | VR1 | VR0 |
    MOVWF   VRCON
    BANKSEL ADCON0		; set the ADC clock, set the input channel AN0 and turn on the ADC
    MOVLW   0b10000001		; | ADCS1 | ADCS0 | CHS3 | CHS2 | CHS1 | CHS0 | GO/DONE | ADON |
    MOVWF   ADCON0
    BANKSEL ADCON1		; set reference voltage source in VDD & VSS ans justify the result to the left
    MOVLW   0b00000000		; | ADFM | xx | VCFG1 | VCFG0 | xx | xx | xx | xx |
    MOVWF   ADCON1
    
    ; TMR0 initialization
    BANKSEL TMR0
    MOVLW   0b00000000
    MOVWF   TMR0
    
    ; ADC initialization
    BANKSEL ADCON0
    BSF     ADCON0, 1		; start ADC conversion (GO/DONE)
    
    ; PORTC initialization
    BANKSEL PORTC
    MOVLW   0b00000000
    MOVWF   PORTC
    
    ; PORTD initialization
    BANKSEL PORTD
    MOVLW   0b00000000
    MOVWF   PORTD
    
    ; variables initialization
    MOVLW   AN0_VALUE		; starting register to store <AN0:AN3> values
    MOVWF   ADC_PORT_IT
    MOVLW   0b11111000		; sensibility range value
    MOVWF   SNSBLTY_RANGE

; main program loop
main:
    
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
    MOVLW   0b00000000		; reset TMR0
    MOVWF   TMR0
    INCF    TMR0_CNTR, F	; increment TMR0 counter variable
    BCF	    INTCON, 2		; clear T0IF bit
    RETURN
    
; interruption subroutine to control ADC
ADCISR:
    
    ; store the value of ADC
    MOVF    ADC_PORT_IT, W
    MOVWF   FSR
    MOVF    ADRESH, W
    MOVWF   INDF
    
    ; increment adc port iterator and reset to 0x30 if necessary
    INCF    ADC_PORT_IT, F
    MOVF    ADC_PORT_IT, W
    SUBLW   ADC_PORT_IT
    BTFSC   STATUS, 2
    GOTO    resetADC_PORT_IT
    GOTO    endADCISR
    
    ; reset ADC port iterator value
    resetADC_PORT_IT:
	MOVLW   AN0_VALUE	; starting register to store <AN0:AN3> values
	MOVWF   ADC_PORT_IT

    ; end ADCISR subroutine
    endADCISR:
    
	; set the next ADC port
	BTFSC   ADC_PORT_IT, 0
	BSF	ADCON0, 2
	BTFSS   ADC_PORT_IT, 0
	BCF	ADCON0, 2
	BTFSC   ADC_PORT_IT, 1
	BSF	ADCON0, 3
	BTFSS   ADC_PORT_IT, 1
	BCF	ADCON0, 3
	
	; end the ADCISR
	BCF	PIR1, 6		; clear ADIF bit
	BSF	ADCON0, 1	; start ADC conversion (GO/DONE)
	RETURN
	
; interruption subroutine to get pressed button in keyboard
keyboardISR:
    
    ; select memory bank 0 <00>
    BCF	    STATUS, 5		; clear RP0 bit
    BCF	    STATUS, 6		; clear RP1 bit
    
    ; clear previous pressed button and found flag
    CLRF    KYBRD_BTN
    CLRF    KYBRD_FND_F
    
    ; search in column 0
    BCF	    PORTD, 4		; active only column 0
    BSF	    PORTD, 5
    BSF	    PORTD, 6
    BSF	    PORTD, 7
    MOVLW   0b00000001		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_FND_F, 0	; if the keyboard found flag is set return
    GOTO    endKeyboardISR
    
    ; search in column 1
    BSF	    PORTD, 4
    BCF	    PORTD, 5		; active only column 1
    BSF	    PORTD, 6
    BSF	    PORTD, 7
    MOVLW   0b00000010		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_FND_F, 0	; if the keyboard found flag is set return
    GOTO    endKeyboardISR
    
    ; search in column 2
    BSF	    PORTD, 4
    BSF	    PORTD, 5
    BCF	    PORTD, 6		; active only column 2
    BSF	    PORTD, 7
    MOVLW   0b00000100		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_FND_F, 0	; if the keyboard found flag is set return
    GOTO    endKeyboardISR
    
    ; search in column 3
    BSF	    PORTD, 4
    BSF	    PORTD, 5
    BSF	    PORTD, 6
    BCF	    PORTD, 7		; active only column 3
    MOVLW   0b00001000		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_FND_F, 0	; if the keyboard found flag is set, return
    GOTO    endKeyboardISR
    
    ; case if there is no match
    CLRF    KYBRD_BTN
    
    ; return from getKeyboard subroutine
    endKeyboardISR:
	CLRF	PORTD
	BCF	INTCON, 1	; clear INTF bit
	BCF	INTCON, 0	; clear RBIF bit
	RETURN
	
; subroutine to find if there are any set bits in the row
searchInRow:
    BTFSS   PORTB, 0
    GOTO    setRow_0
    BTFSS   PORTB, 1
    GOTO    setRow_1
    BTFSS   PORTB, 2
    GOTO    setRow_2
    BTFSS   PORTB, 3
    GOTO    setRow_3
    RETURN
    setRow_0:
	BSF	KYBRD_BTN, 4
	BSF	KYBRD_FND_F, 0
	RETURN
    setRow_1:
	BSF	KYBRD_BTN, 5
	BSF	KYBRD_FND_F, 0
	RETURN
    setRow_2:
	BSF	KYBRD_BTN, 6
	BSF	KYBRD_FND_F, 0
	RETURN
    setRow_3:
	BSF	KYBRD_BTN, 7
	BSF	KYBRD_FND_F, 0
	RETURN
	
; subroutine to convert a value in W by performing additions based on bit positions
kybrdToHexConv:
    
    ; select memory bank 0 <00>
    BCF	    STATUS, 5		; clear RP0 bit
    BCF	    STATUS, 6		; clear RP1 bit
    
    ; clear W
    CLRW
    
    ; add bit 7 (accumulator + 0)
    BTFSS   KYBRD_BTN, 7
    GOTO    addBit_6
    ADDLW   0x00		; value to add
    
    ; add bit 6 (accumulator + 1)
    addBit_6:
	BTFSS   KYBRD_BTN, 6
	GOTO    addBit_5
	ADDLW   0x01		; value to add

    ; add bit 5 (accumulator + 2)
    addBit_5:
	BTFSS   KYBRD_BTN, 5
	GOTO    addBit_4
	ADDLW   0x02		; value to add
	
    ; add bit 4 (accumulator + 3)
    addBit_4:
	BTFSS   KYBRD_BTN, 4
	GOTO    addBit_3
	ADDLW   0x03		; value to add
	
    ; add bit 3 (accumulator + 0)
    addBit_3:
	BTFSS   KYBRD_BTN, 3
	GOTO    addBit_2
	ADDLW   0x00		; value to add
	
    ; add bit 2 (accumulator + 4)
    addBit_2:
	BTFSS   KYBRD_BTN, 2
	GOTO    addBit_1
	ADDLW   0x04		; value to add
	
    ; add bit 1 (accumulator + 8)
    addBit_1:
	BTFSS   KYBRD_BTN, 1
	GOTO    addBit_0
	ADDLW   0x08		; value to add
	
    ; add bit 0 (accumulator + 12)
    addBit_0:
	BTFSS   KYBRD_BTN, 0
	GOTO    addNULL
	ADDLW   0x0C		; value to add

    ; add null
    addNULL:
	MOVWF	KYBRD_BTN
	RETURN
    
; subroutine to control the up/down movement of stepper motor
moveUpDown:
    
    ; compare the measured voltages
    MOVF    AN0_VALUE, W
    SUBWF   AN1_VALUE, W	; subtract AN1_VALUE from AN0_VALUE
    
    ; apply sensibility value
    ANDWF   SNSBLTY_RANGE, W
    
    ; rotate up or down if necessary
    BTFSC   STATUS, 2		; if the result is zero, do nothing
    GOTO    stopRotUD
    BTFSS   STATUS, 0		; if the result is positive, rotate up
    GOTO    rotUp
    BTFSC   STATUS, 0		; if the result is negative, rotate down
    GOTO    rotDown
    
    ; rotate one step up
    rotUp:
	BSF	PORTC, 0	; set direction (RC0) in HIGH
	BSF	PORTC, 1	; set pulse (RC1) in HIGH
	CALL	getDelay
	BCF	PORTC, 1	; set pulse (RC1) in LOW
	CALL	getDelay
	RETURN
    
    ; rotate one step down
    rotDown:
	BCF	PORTC, 0	; set direction (RC0) in LOW
	BSF	PORTC, 1	; set pulse (RC1) in HIGH
	CALL	getDelay
	BCF	PORTC, 1	; set pulse (RC1) in LOW
	CALL	getDelay
	RETURN
    
    ; no rotation
    stopRotUD:
	BCF	PORTC, 0	; set direction (RC0) in LOW
	BCF	PORTC, 1	; set pulse (RC1) in LOW
	CALL	getDelay
	CALL	getDelay	; make two delays to complete one cycle
	RETURN
	
; subroutine to control the left/right movement of stepper motor
moveLeftRight:
    
    ; compare the measured voltages
    MOVF    AN2_VALUE, W
    SUBWF   AN3_VALUE, W	; subtract AN3_VALUE from AN2_VALUE
    
    ; apply sensibility value
    ANDWF   SNSBLTY_RANGE, W
    
    ; rotate left or right if necessary
    BTFSC   STATUS, 2		; if the result is zero, do nothing
    GOTO    stopRotLR
    BTFSS   STATUS, 0		; if the result is positive, rotate left
    GOTO    rotLeft
    BTFSC   STATUS, 0		; if the result is negative, rotate right
    GOTO    rotRight
    
    ; rotate one step up
    rotLeft:
	BSF	PORTC, 2	; set direction (RC2) in HIGH
	BSF	PORTC, 3	; set pulse (RC3) in HIGH
	CALL	getDelay
	BCF	PORTC, 3	; set pulse (RC3) in LOW
	CALL	getDelay
	RETURN
    
    ; rotate one step down
    rotRight:
	BCF	PORTC, 2	; set direction (RC2) in LOW
	BSF	PORTC, 3	; set pulse (RC3) in HIGH
	CALL	getDelay
	BCF	PORTC, 3	; set pulse (RC3) in LOW
	CALL	getDelay
	RETURN
    
    ; no rotation
    stopRotLR:
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