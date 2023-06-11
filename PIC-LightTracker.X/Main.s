#include "xc.inc"

; CONFIG1
  CONFIG  FOSC = INTRC_NOCLKOUT ; Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
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
    BTFSS   INTCON, 0		; check RBIF bit
    GOTO    $+3
    CALL    limitSwitchsISR
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
TMR0_CNTR	EQU 0x28	; TMR0 counter
TMR0_CNTR_REF	EQU 0x29	; TMR0 counter temporary reference

; ADC
AN0_VALUE	EQU 0x30
AN1_VALUE	EQU 0x31
AN2_VALUE	EQU 0x32
AN3_VALUE	EQU 0x33
ADC_PORT_IT	EQU 0x34	; ADC port iterator
SNSBLTY_RANGE	EQU 0x35	; sensibility range to prevent oscilations

; keyboard
KYBRD_BTN	EQU 0x38	; store the pressed button
KYBRD_F		EQU 0x39	; keyboard flags

; limit switchs
LIMIT_SW_F	EQU 0X40	; limit switchs flags

; stepper motors
MOTOR_POS_0L	EQU 0x48	; stepper motor 0 position low value
MOTOR_POS_0H	EQU 0x49	; stepper motor 0 position high value
MOTOR_POS_1L	EQU 0x4A	; stepper motor 1 position low value
MOTOR_POS_1H	EQU 0x4B	; stepper motor 1 position high value
STEP_CNTR_AUX	EQU 0x4C	; auxiliary step counter

; light tracker operation modes
OP_MODE		EQU 0x50	; operation mode

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
    CLRF    ANSELH		; set <ANS8:ANS13> as digitals
    
    ; PORTC configuration (USART)
    BANKSEL TRISC
    MOVLW   0b11000000		; set <RC6:RC7> as inputs
    MOVWF   TRISC
    
    ; PORTD configuration (stepper motors & keyboard columns)
    BANKSEL TRISD
    CLRF    TRISD		; set <RD0:RD7> as outputs
    
    ; general ports configurations
    BANKSEL OPTION_REG		; enable global pull-ups and set pre-scaler (100=fast, 110=slow) 
    MOVLW   0b00000100		; | /RBPU | INTEDG | T0CS | T0SE | PSA | PS2 | PS1 | PS0 |
    MOVWF   OPTION_REG
    BANKSEL WPUB
    MOVLW   0b11111111		; enable pull-ups in <RB0:RB7>
    MOVWF   WPUB
    
    ; ADC configuration
    BANKSEL VRCON		; set the reference voltage
    CLRF    VRCON		; | VREN | VROE | VRR | VRSS | VR3 | VR2 | VR1 | VR0 |
    BANKSEL ADCON0		; set the ADC clock, set the input channel AN0 and turn on the ADC
    MOVLW   0b10000001		; | ADCS1 | ADCS0 | CHS3 | CHS2 | CHS1 | CHS0 | GO/DONE | ADON |
    MOVWF   ADCON0
    BANKSEL ADCON1		; set reference voltage source in VDD & VSS ans justify the result to the left
    CLRF    ADCON1		; | ADFM | xx | VCFG1 | VCFG0 | xx | xx | xx | xx |
    
    ; EUSART configuration
    BANKSEL TXSTA
    MOVLW   0b00100010		; | CSRC | TX9 | TXEN | SYNC | SENDB | BRGH | TRMT | TX9D |
    MOVWF   TXSTA
    BANKSEL RCSTA
    MOVLW   0b10010000		; | SPEN | RX9 | SREN | CREN | ADDEN | FERR | OERR | RX9D |
    MOVWF   RCSTA
    BANKSEL SPBRG
    MOVLW   0xCF		; set the baud rate generator
    MOVWF   SPBRG
    
    ; interruptions configuration
    BANKSEL INTCON		; enable  interruptions in PEIE, interruptions in TMR0 and interruptions in PORTC
    MOVLW   0b01111000		; | GIE | PEIE | T0IE | INTE | RBIE | T0IF | INTF | RBIF |
    MOVWF   INTCON
    BANKSEL IOCB
    MOVLW   0b11111111		; enable interruptions in <RB0:RB7>
    MOVWF   IOCB
    BANKSEL PIE1		; enable interruptions in ADC and in EUSART receive
    MOVLW   0b01100000		; | xx | ADIE | RCIE | TXIE | SSPIE | CCP1IE | TMR2IE | TMR1IE |
    MOVWF   PIE1
    
    ; PORTD initialization
    BANKSEL PORTD
    CLRF    PORTD		; set PORTD in LOW
    
    ; TMR0 initialization
    BANKSEL TMR0
    CLRF    TMR0		; set initial value for TMR0
    
    ; ADC initialization
    BANKSEL ADCON0
    BSF     ADCON0, 1		; start ADC conversion (GO/DONE)
    
    ; interruptions initialization
    BANKSEL INTCON
    BSF	    INTCON, 7		; enable global interruptions
    
    ; select memory bank 0 <00>
    BCF	    STATUS, 5		; clear RP0 bit
    BCF	    STATUS, 6		; clear RP1 bit
    
    ; variables initialization
    MOVLW   AN0_VALUE		; starting register to store <AN0:AN3> values
    MOVWF   ADC_PORT_IT
    MOVLW   0b11110000		; sensibility range value
    MOVWF   SNSBLTY_RANGE
    CLRF    KYBRD_BTN
    CLRF    KYBRD_F
    CLRF    LIMIT_SW_F
    CLRF    OP_MODE
    
    ; axis recognition sequence
;    CALL    moveUpToLimitSw
;    CALL    getDelay
;    CALL    moveDownToLimitSw
;    CALL    getDelay
;    MOVLW   0xAC
;    MOVWF   STEP_CNTR_AUX
;    CALL    rotUp
;    DECFSZ  STEP_CNTR_AUX
;    GOTO    $-2
;    CALL    moveLeftToLimitSw
;    CALL    getDelay
;    CALL    moveRightToLimitSw
;    CALL    getDelay
;    MOVLW   0xD0
;    MOVWF   STEP_CNTR_AUX
;    CALL    rotLeft
;    DECFSZ  STEP_CNTR_AUX
;    GOTO    $-2

; main program loop
main:
    
    ; select memory bank 0 <00>
    BCF	    STATUS, 5		; clear RP0 bit
    BCF	    STATUS, 6		; clear RP1 bit
    
    ; EUSART message check
    MOVLW   0x48		; ASCII 'H'
    CALL    EUSARTsend
    MOVLW   0x4F		; ASCII 'O'
    CALL    EUSARTsend
    MOVLW   0x4C		; ASCII 'L'
    CALL    EUSARTsend
    MOVLW   0x41		; ASCII 'A'
    CALL    EUSARTsend
    
    ; set the operation mode
    MOVF    KYBRD_BTN, W
    BTFSS   STATUS, 2
    MOVWF   OP_MODE
    
    ; if OP_MODE is 0x03 call lightTrackerMode
    MOVF    OP_MODE, W
    SUBLW   0x03
    BTFSC   STATUS, 2
    CALL    lightTrackerMode
    
    ; if OP_MODE is 0x0A call rotate up
    MOVF    OP_MODE, W
    SUBLW   0x0A
    BTFSC   STATUS, 2
    CALL    rotUp
    
    ; if OP_MODE is 0x08 call rotate down
    MOVF    OP_MODE, W
    SUBLW   0x08
    BTFSC   STATUS, 2
    CALL    rotDown
    
    ; if OP_MODE is 0x05 call rotate left
    MOVF    OP_MODE, W
    SUBLW   0x05
    BTFSC   STATUS, 2
    CALL    rotLeft
    
    ; if OP_MODE is 0x0D call rotate right
    MOVF    OP_MODE, W
    SUBLW   0x0D
    BTFSC   STATUS, 2
    CALL    rotRight
    
    ; if OP_MODE is 0x09 do nothing
    MOVF    OP_MODE, W
    SUBLW   0x09
    BTFSC   STATUS, 2
    CALL    getDelay
    
    GOTO    main
    
; EUSART transmit pre-loaded value in W
EUSARTsend:
    BANKSEL TXREG
    MOVWF   TXREG		; load the W data into TXREG
    BANKSEL TXSTA
    BTFSC   TXSTA, 1		; check if the data has been sent
    GOTO    $-1			; loop until the data has been sent
    RETURN

; EUSART receive
EUSARTreceiveISR:
    RETURN
    
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
    
    ; increment adc port iterator and reset to AN1_VALUE position if necessary
    INCF    ADC_PORT_IT, F
    MOVF    ADC_PORT_IT, W
    SUBLW   ADC_PORT_IT
    BTFSC   STATUS, 2		; if the port equals ADC_PORT_IT, then reset the iterator
    GOTO    $+2
    GOTO    $+3
    
    ; reset ADC port iterator value
    MOVLW   AN0_VALUE		; starting register to store <AN0:AN3> values
    MOVWF   ADC_PORT_IT
    
    ; set the next ADC port
    BTFSC   ADC_PORT_IT, 0
    BSF	    ADCON0, 2
    BTFSS   ADC_PORT_IT, 0
    BCF	    ADCON0, 2
    BTFSC   ADC_PORT_IT, 1
    BSF	    ADCON0, 3
    BTFSS   ADC_PORT_IT, 1
    BCF	    ADCON0, 3
    
    ; end of ADCISR
    BCF	    PIR1, 6		; clear ADIF bit
    BSF	    ADCON0, 1		; start ADC conversion (GO/DONE)
    RETURN

; interruption subroutine to manage limit switchs
limitSwitchsISR:
    
    ; up limit switch
    BTFSS   PORTB, 4
    BSF	    LIMIT_SW_F, 0
    BTFSC   PORTB, 4
    BCF	    LIMIT_SW_F, 0
    
    ; down limit switch
    BTFSS   PORTB, 5
    BSF	    LIMIT_SW_F, 1
    BTFSC   PORTB, 5
    BCF	    LIMIT_SW_F, 1
    
    ; left limit switch
    BTFSS   PORTB, 6
    BSF	    LIMIT_SW_F, 2
    BTFSC   PORTB, 6
    BCF	    LIMIT_SW_F, 2
    
    ; right limit switch
    BTFSS   PORTB, 7
    BSF	    LIMIT_SW_F, 3
    BTFSC   PORTB, 7
    BCF	    LIMIT_SW_F, 3
    
    ; end of limitSwitchsISR
    BCF	    INTCON, 1	; clear INTF bit
    BCF	    INTCON, 0	; clear RBIF bit
    RETURN
    
; interruption subroutine to get pressed button in keyboard
keyboardISR:
    
    ; select memory bank 0 <00>
    BCF	    STATUS, 5		; clear RP0 bit
    BCF	    STATUS, 6		; clear RP1 bit
    
    ; clear previous pressed button and found flag
    CLRF    KYBRD_BTN
    CLRF    KYBRD_F
    
    ; search in column 0
    BCF	    PORTD, 4		; active only column 0
    BSF	    PORTD, 5
    BSF	    PORTD, 6
    BSF	    PORTD, 7
    MOVLW   0b00000001		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_F, 0		; if the keyboard found flag is set go to end
    GOTO    $+29
    
    ; search in column 1
    BSF	    PORTD, 4
    BCF	    PORTD, 5		; active only column 1
    BSF	    PORTD, 6
    BSF	    PORTD, 7
    MOVLW   0b00000010		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_F, 0		; if the keyboard found flag is set go to end
    GOTO    $+20
    
    ; search in column 2
    BSF	    PORTD, 4
    BSF	    PORTD, 5
    BCF	    PORTD, 6		; active only column 2
    BSF	    PORTD, 7
    MOVLW   0b00000100		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_F, 0		; if the keyboard found flag is set go to end
    GOTO    $+11
    
    ; search in column 3
    BSF	    PORTD, 4
    BSF	    PORTD, 5
    BSF	    PORTD, 6
    BCF	    PORTD, 7		; active only column 3
    MOVLW   0b00001000		; save current column
    MOVWF   KYBRD_BTN
    CALL    searchInRow		; find if the row is found with the current column
    BTFSC   KYBRD_F, 0		; if the keyboard found flag is set go to end
    GOTO    $+2
    
    ; case if there is no match
    CLRF    KYBRD_BTN
    
    ; end of keyboardISR
    CALL    kybrdToHexConv
    BCF	    PORTD, 4
    BCF	    PORTD, 5
    BCF	    PORTD, 6
    BCF	    PORTD, 7
    BCF	    INTCON, 1		; clear INTF bit
    BCF	    INTCON, 0		; clear RBIF bit
    RETURN
	
; subroutine to find if there are any set bits in the row
searchInRow:
    BTFSS   PORTB, 0
    GOTO    $+8
    BTFSS   PORTB, 1
    GOTO    $+9
    BTFSS   PORTB, 2
    GOTO    $+10
    BTFSS   PORTB, 3
    GOTO    $+11
    RETURN
    
    ; set the row 0
    BSF	    KYBRD_BTN, 4
    BSF	    KYBRD_F, 0
    RETURN
    
    ; set the row 1
    BSF	    KYBRD_BTN, 5
    BSF	    KYBRD_F, 0
    RETURN
    
    ; set the row 2
    BSF	    KYBRD_BTN, 6
    BSF	    KYBRD_F, 0
    RETURN
    
    ; set the row 3
    BSF	    KYBRD_BTN, 7
    BSF	    KYBRD_F, 0
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
    GOTO    $+2
    ADDLW   0x00		; value to add
    
    ; add bit 6 (accumulator + 1)
    BTFSS   KYBRD_BTN, 6
    GOTO    $+2
    ADDLW   0x01		; value to add
    
    ; add bit 5 (accumulator + 2)
    BTFSS   KYBRD_BTN, 5
    GOTO    $+2
    ADDLW   0x02		; value to add
    
    ; add bit 4 (accumulator + 3)
    BTFSS   KYBRD_BTN, 4
    GOTO    $+2
    ADDLW   0x03		; value to add
    
    ; add bit 3 (accumulator + 0)
    BTFSS   KYBRD_BTN, 3
    GOTO    $+2
    ADDLW   0x00		; value to add
    
    ; add bit 2 (accumulator + 4)
    BTFSS   KYBRD_BTN, 2
    GOTO    $+2
    ADDLW   0x04		; value to add
    
    ; add bit 1 (accumulator + 8)
    BTFSS   KYBRD_BTN, 1
    GOTO    $+2
    ADDLW   0x08		; value to add
    
    ; add bit 0 (accumulator + 12)
    BTFSS   KYBRD_BTN, 0
    GOTO    $+2
    ADDLW   0x0C		; value to add
    
    ; add null
    MOVWF   KYBRD_BTN
    RETURN
    
; subroutine to move the stepper motor up to the limit switch
moveUpToLimitSw:
    CALL    rotUp
    BTFSS   LIMIT_SW_F, 0
    GOTO    $-2
    
    ; set the zero position of the stepper motor
    CLRF    MOTOR_POS_0L
    CLRF    MOTOR_POS_0H
    RETURN
    
; subroutine to move the stepper motor down to the limit switch
moveDownToLimitSw:
    CALL    rotDown
    BTFSS   LIMIT_SW_F, 1
    GOTO    $-2
    RETURN
    
; subroutine to move the stepper motor left to the limit switch
moveLeftToLimitSw:
    CALL    rotLeft
    BTFSS   LIMIT_SW_F, 2
    GOTO    $-2
    
    ; set the zero position of the stepper motor
    CLRF    MOTOR_POS_1L
    CLRF    MOTOR_POS_1H
    RETURN
    
; subroutine to move the stepper motor right to the limit switch
moveRightToLimitSw:
    CALL    rotRight
    BTFSS   LIMIT_SW_F, 3
    GOTO    $-2
    RETURN
    
; subroutine to move the light tracker using light
lightTrackerMode:
    
    ; compare the measured voltages
    MOVF    AN0_VALUE, W
    SUBWF   AN1_VALUE, W	; subtract AN1_VALUE from AN0_VALUE
    ANDWF   SNSBLTY_RANGE, W	; apply sensibility value
    
    ; rotate up or down if necessary
    BTFSC   STATUS, 2		; if the result is zero, do nothing
    GOTO    $+9
    BTFSC   STATUS, 0		; if the result is negative, rotate up
    GOTO    $+3
    BTFSS   STATUS, 0		; if the result is positive, rotate down
    GOTO    $+3
    
    ; rotation options
    CALL    rotUp
    GOTO    $+4
    CALL    rotDown
    GOTO    $+2
    CALL    stopRotUD
    
    ; compare the measured voltages
    MOVF    AN2_VALUE, W
    SUBWF   AN3_VALUE, W	; subtract AN3_VALUE from AN2_VALUE
    ANDWF   SNSBLTY_RANGE, W	; apply sensibility value
    
    ; rotate left or right if necessary
    BTFSC   STATUS, 2		; if the result is zero, do nothing
    GOTO    $+9
    BTFSC   STATUS, 0		; if the result is negative, rotate left
    GOTO    $+3
    BTFSS   STATUS, 0		; if the result is positive, rotate right
    GOTO    $+3
    
    ; rotation options
    CALL    rotLeft
    GOTO    $+4
    CALL    rotRight
    GOTO    $+2
    CALL    stopRotLR
    RETURN

; subroutine to rotate one step up
rotUp:
    BCF	    PORTD, 0		; set direction (RD0) in LOW
    BTFSS   LIMIT_SW_F, 0	; if the limit switch is in HIGH, dont send pulse
    BSF	    PORTD, 1		; set pulse (RD1) in HIGH
    CALL    getDelay
    BCF	    PORTD, 1		; set pulse (RD1) in LOW
    CALL    getDelay
    
    ; increment stepper motor position
    BTFSC   LIMIT_SW_F, 0	; if the limit switch is in HIGH, don't increment position
    GOTO    $+4
    INCF    MOTOR_POS_0L, F
    BTFSC   STATUS, 0		; if there is carry, increment MOTOR_POS_0H
    INCF    MOTOR_POS_0H, F
    RETURN
    
; subroutine to rotate one step down
rotDown:
    BSF	    PORTD, 0		; set direction (RD0) in HIGH
    BTFSS   LIMIT_SW_F, 1	; if the limit switch is in HIGH, dont send pulse
    BSF	    PORTD, 1		; set pulse (RD1) in HIGH
    CALL    getDelay
    BCF	    PORTD, 1		; set pulse (RD1) in LOW
    CALL    getDelay
    
    ; decrement stepper motor position
    BTFSC   LIMIT_SW_F, 1	; if the limit switch is in HIGH, don't increment position
    GOTO    $+5
    MOVF    MOTOR_POS_0L
    BTFSC   STATUS, 2		; if it is zero, decrement MOTOR_POS_0H
    DECF    MOTOR_POS_0H, F
    DECF    MOTOR_POS_0L, F
    RETURN

; subroutine to no rotation
stopRotUD:
    BCF	    PORTD, 0		; set direction (RD0) in LOW
    BCF	    PORTD, 1		; set pulse (RD1) in LOW
    CALL    getDelay
    CALL    getDelay		; make two delays to complete one cycle
    RETURN

; rotate one step up
rotLeft:
    BCF	    PORTD, 2		; set direction (RD2) in LOW
    BTFSS   LIMIT_SW_F, 2	; if the limit switch is in HIGH, dont send pulse
    BSF	    PORTD, 3		; set pulse (RD3) in HIGH
    CALL    getDelay
    BCF	    PORTD, 3		; set pulse (RD3) in LOW
    CALL    getDelay
    
    ; increment stepper motor position
    BTFSC   LIMIT_SW_F, 2	; if the limit switch is in HIGH, don't increment position
    GOTO    $+4
    INCF    MOTOR_POS_1L, F
    BTFSC   STATUS, 0		; if there is carry, increment MOTOR_POS_1H
    INCF    MOTOR_POS_1H, F
    RETURN

; rotate one step down
rotRight:
    BSF	    PORTD, 2		; set direction (RD2) in HIGH
    BTFSS   LIMIT_SW_F, 3	; if the limit switch is in HIGH, dont send pulse
    BSF	    PORTD, 3		; set pulse (RD3) in HIGH
    CALL    getDelay
    BCF	    PORTD, 3		; set pulse (RD3) in LOW
    CALL    getDelay
    
    ; decrement stepper motor position
    BTFSC   LIMIT_SW_F, 3	; if the limit switch is in HIGH, don't increment position
    GOTO    $+5
    MOVF    MOTOR_POS_1L
    BTFSC   STATUS, 2		; if it is zero, decrement MOTOR_POS_1H
    DECF    MOTOR_POS_1H, F
    DECF    MOTOR_POS_1L, F
    RETURN

; no rotation
stopRotLR:
    BCF	    PORTD, 2		; set direction (RD2) in LOW
    BCF	    PORTD, 3		; set pulse (RD3) in LOW
    CALL    getDelay
    CALL    getDelay		; make two delays to complete one cycle
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