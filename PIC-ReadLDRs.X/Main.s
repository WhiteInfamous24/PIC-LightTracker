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
W_TMP	    EQU 0x20
STATUS_TMP  EQU	0x21
AN0_VALUE   EQU 0x22
AN1_VALUE   EQU 0x23

; program setup
setup:
    
    ; ports configuration
    BANKSEL TRISA
    MOVLW   0b00000011		; set AN0 and AN1 as inputs
    MOVWF   TRISA
    BANKSEL TRISB
    MOVLW   0b00000000		; set RB0, RB1, RB2 & RB3 as outputs
    MOVWF   TRISB
    BANKSEL ANSEL
    MOVLW   0b00000011		; enable analog inputs on AN0 and AN1
    MOVWF   ANSEL

    ; ADC configuration
    BANKSEL VRCON		; set the reference voltage
    MOVLW   0b00000000		; VREN - VROE - VRR - VRSS - VR3 - VR2 - VR1 - VR0
    MOVWF   VRCON
    BANKSEL ADCON0		; set the clock, set the input channel AN0 and turn on the ADC
    MOVLW   0b10000001		; ADCS1 - ADCS0 - CHS3 - CHS2 - CHS1 - CHS0 - GO/DONE - ADON
    MOVWF   ADCON0
    BANKSEL ADCON1		; select the reference voltage source (VDD and VSS)
    MOVLW   0b00000000		; ADFM - xx - VCFG1 - VCFG0 - xx - xx - xx - xx
    MOVWF   ADCON1

; main program loop
main:
    
    ; switch to channel AN0 and measure voltage on pin AN0
    BANKSEL ADCON0
    BCF	    ADCON0, 2		; set the ADC to measure voltage on pin AN0
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete
    GOTO    $-1
    MOVF    ADRESL, 0		; read the conversion result in ADRESL
    MOVWF   AN0_VALUE		; store the result in the variable AN0_VALUE

    ; switch to channel AN1 and measure voltage on pin AN1
    BANKSEL ADCON0
    BSF	    ADCON0, 2		; set the ADC to measure voltage on pin AN1
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete (GO/DONE)
    GOTO    $-1
    MOVF    ADRESL, 0		; read the conversion result in ADRESL
    MOVWF   AN1_VALUE		; store the result in the variable AN1_VALUE

    ; compare the measured voltages and turn on the corresponding LEDs
    MOVF    AN0_VALUE, 0
    SUBWF   AN1_VALUE, 0	; subtract AN1_VALUE from AN0_VALUE
    BTFSC   STATUS, 2		; if the result is zero, all LEDs turn off
    GOTO    turnOffLEDs
    BTFSC   STATUS, 2
    GOTO    $+5
    BTFSS   STATUS, 0		; if the result is positive, turn on the LED in RB0
    CALL    turnOnLEDRB0
    BTFSC   STATUS, 0		; if the result is negative, turn on the LED in RB1
    CALL    turnOnLEDRB1
    
    GOTO    main

; subroutine to turn off all LEDs
turnOffLEDs:
    MOVLW   0b00000000
    MOVWF   PORTB
    
    RETURN

; subroutine to light the LED in RB0
turnOnLEDRB0:
    MOVLW   0b00000001
    MOVWF   PORTB
    
    RETURN

; subroutine to light the LED in RB1
turnOnLEDRB1:
    MOVLW   0b00000010
    MOVWF   PORTB
    
    RETURN

END RESET_VECT