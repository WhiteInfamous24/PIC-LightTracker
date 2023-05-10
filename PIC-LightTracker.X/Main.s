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
W_REG		EQU 0
F_REG		EQU 1

; program setup
setup:
    
    ; configuration of inputs/outputs pins
    BANKSEL TRISA
    MOVLW   0b00001111		; configuration of the first 4 pins of port-A as analog input
    MOVWF   TRISA
    BANKSEL TRISB
    MOVLW   0b00000000		; configuration of the first 4 pins of port-B as digital output
    MOVWF   TRISB
    
    ; configuration of ADC
    BANKSEL ADCON0
    MOVLW   0b10000001		; turn on the ADC and select channel AN0 as input
    MOVWF   ADCON0
    MOVLW   0b00000110		; set the reference voltage in VDD and VSS
    MOVWF   ADCON1
    
    ; output pins configuration for stepper motors
    BANKSEL PORTB
    MOVLW   0b00000000		; initialize port-B pins as low output
    MOVWF   PORTB

    ; configuration of stepper motor parameters
    MOVWF   STP_STEPS
    MOVWF   STP_DELAY
    MOVWF   STP_ACCEL
    MOVWF   STP_MAXSPEED
    MOVWF   STP_MINSPEED

; main program loop
main:
    
    ; read the value of the analog input AN0
    BANKSEL ADCON0
    BSF	    ADCON0, GO		; start conversion
waitForADC:
    BTFSC   ADCON0, GO		; wait for conversion to finish
    GOTO    waitForADC
    MOVF    ADRESH, W		; read the converted value
    BANKSEL PORTB

    ; control stepper motors according to light intensity measured by LDR sensors
    ; stepper motor 1
    MOVF    ADRESH, W		; read the converted value
    SUBWF   #SENSITIVITY_1, W	; subtract sensor sensitivity 1
    BTFSC   STATUS, C		; if the result is negative, sensor 1 is dark
    CALL    rotateLeft		; rotate the stepper motor to the left
    BTFSS   STATUS, C		; if the result is positive, sensor 1 is illuminated
    CALL    rotateRight		; rotate the stepper motor to the right
    
    ; stepper motor 2
    MOVF    ADRESH, W		; read the converted value
    SUBWF   #SENSITIVITY_2, W	; subtract sensor sensitivity 2
    BTFSC   STATUS, C		; if the result is negative, sensor 2 is dark
    CALL    rotateUp		; rotate the stepper motor up
    BTFSS   STATUS, C		; if the result is positive, sensor 2 is illuminated
    CALL    rotateDown		; rotate the stepper motor down
    
    ; return to main program loop
    GOTO    main
    
rotateLeft:
    
    ; IMPLEMENT METHOD
    
    RETURN
    
rotateRight:
    
    ; IMPLEMENT METHOD
    
    RETURN
    
rotateUp:
    
    ; IMPLEMENT METHOD
    
    RETURN
    
rotateDown:
    
    ; IMPLEMENT METHOD
    
    RETURN

END RESET_VECT