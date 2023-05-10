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

; starting position of the program
psect RESET_VECT, class=CODE, delta=2
RESET_VECT:
    GOTO    setup

; memory location to go when a interrupt happens
psect INT_VECT, class=CODE, delta=2
INT_VECT:
    
    ; IMPLEMENT METHOD INTERRUPTION
    
    RETFIE

; program variables
AN0_VALUE   EQU 0x20
AN1_VALUE   EQU 0x21

; program setup
setup:
    
    ; port configuration
    BANKSEL TRISA
    MOVLW   0b00000011		; set AN0 and AN1 as inputs
    MOVWF   TRISA
    BANKSEL TRISB
    CLRF    TRISB		; set RB0 and RB1 as outputs
    BANKSEL ANSELH
    MOVLW   0b00000011		; enable analog inputs on AN0 and AN1
    MOVWF   ANSELH

    ; ADC configuration
    BANKSEL ADCON1
    CLRF    ADCON1		; set all pins as analog inputs
    MOVLW   0b00000010		; select the reference voltage source (VDD and VSS)
    BANKSEL ADCON0
    MOVWF   ADCON0		; set the ADC in single conversion mode and select channel AN0

; main program loop
main:
    
    ; turn off the LEDs
    BCF	    PORTB, 0		; turn off the LED on RB0
    BCF	    PORTB, 1		; turn off the LED on RB1
    
    ; measure the voltage on pin AN0
    BANKSEL ADCON0
    BSF     ADCON0, 1		; start conversion
    BTFSC   ADCON0, 1		; wait until the conversion is complete
    GOTO    $-1
    MOVF    ADRESH, 0		; read the conversion result in ADRESH and ADRESL
    MOVWF   AN0_VALUE		; store the result in the variable AN0_VALUE

    ; switch to channel AN1 and measure voltage on pin AN1
    BANKSEL ADCON0
    MOVLW   0b00000011		; select channel AN1
    MOVWF   ADCON0		; set the ADC to measure voltage on pin AN1
    BSF     ADCON0, 1		; start conversion (GO/DONE)
    BTFSC   ADCON0, 1		; wait until the conversion is complete (GO/DONE)
    GOTO    $-1
    MOVF    ADRESH, 0		; read the conversion result in ADRESH and ADRESL
    MOVWF   AN1_VALUE		; store the result in the variable AN1_VALUE

    ; compare the measured voltages and turn on the corresponding LEDs
    MOVF    AN0_VALUE, 0
    SUBWF   AN1_VALUE, 0	; subtract AN1_VALUE from AN0_VALUE
    BTFSC   STATUS, 0		; if the result is positive or zero, turn on the LED in RB0
    BSF     PORTB, 0
    BTFSS   STATUS, 0		; if the result is negative, turn on the LED on RB1
    BSF     PORTB, 1
    
    ; return to main program loop
    GOTO    main

END RESET_VECT