 ;*******************************************************************************
 ; en0627_assignment.asm
 ; Authors....: Mitchell Smith (w10017351), Adam Brown (w10015647)
 ; Revision...: 1.01
 ; Date.......: 2015-01-28
 ;
 ;   Description
 ;
 ; Designed to meet the specification given.
 ; Calculates distance using result from Polaroid 6500 ultrasonic sensor
 ; then determines if within acceptable limits and lights LEDs depending on
 ; result
 ;
 ; Includes a menu that is printed to LCD for human-device interface
 ;
 ;   Pinout
 ;                   _________
 ;       RA2 / DB6 -| 16f1847 |- DB5 / RA1
 ;       RA3 / DB7 -|         |- DB4 / RA0
 ;        RA4 / RS -|         |- EN / RA7
 ;       RA5 / N/A -|         |- RW / RA6
 ;             Vss -|         |- Vdd
 ;      RB0 / ECHO -|         |- RIGHT / RB7
 ;      RB1 / INIT -|         |- LEFT / RB6
 ;   RB2 / RED_LED -|         |- ENTER / RB5
 ; RB3 / AMBER_LED -|_________|- GREEN_LED / RB4
 ;
 
 list p=16F1847
 #include    <p16F1847.inc>
 
 ; +++ CONSTANTS +++
 DIST_MAX        EQU .1300 ; The maximum distance we calculate is 2000mm, 0x7D0
 DIST_MIN        EQU .0500 ; The mimimum distance we calculate is 400mm, 0x190
 PASSBAND_MAX    EQU .0875 ; 0x032A
 PASSBAND_MIN    EQU .0800 ; 0x02DF
 TOLERANCE_MAX   EQU .0919 ; PASSBAND_MIX + 5% 0x0397
 TOLERANCE_MIN   EQU .0760 ; PASSBAND_MIN - 5% 0x02F8
 
 MM_PER_US   EQU 0x00AE ; (mm per second * tinstruction * 1024) = .174
 PULSEGEN    EQU 0x0000 ;0x01B8;\ The time it takes for ultrasound to generate pulses
 LATENCY     EQU 0x0000 ;0x00C8    ;\ 200mm
 FAIL        EQU 0
 PASS        EQU 1
 TOLERANCE   EQU 2
 TIMEOUT     EQU 3
 
 ; The maximum value of menu_selected. Must be equal to
 ; limit = n +1 (eg options 1...3 .'. n = 3 + 1 = 4)
 START_MENU_MAX              EQU .6
 UPPER_PASSBAND_MENU_MAX     EQU .8
 LOWER_PASSBAND_MENU_MAX     EQU .8
 UPPER_TOLERANCE_MENU_MAX    EQU .8
 LOWER_TOLERANCE_MENU_MAX    EQU .8
 
 ; The minimum value of menu_selected for menus.
 START_MENU_MIN              EQU .2
 LIMIT_MENU_MIN              EQU .2
 UPPER_PASSBAND_MENU_MIN     EQU .2
 LOWER_PASSBAND_MENU_MIN     EQU .2
 TOLERANCE_MENU_MIN          EQU .2
 UPPER_TOLERANCE_MENU_MIN    EQU .2
 LOWER_TOLERANCE_MENU_MIN    EQU .2
 
 ; Hardware Assignment
 LCDPORT EQU PORTA
 LCDDIR  EQU TRISA
 DB4     EQU .0
 DB5     EQU .1
 DB6     EQU .2
 DB7     EQU .3
 RS      EQU .4
 RW      EQU .6
 EN      EQU .7
 
 USPORT  EQU PORTB
 USDIR   EQU TRISB
 USPU    EQU WPUB
 ECHO    EQU .0 ; RB.0 (Input, internal pullup)
 INIT    EQU .1 ; RB.1 (Output)
 
 LEDPORT     EQU PORTB
 LEDDIR      EQU PORTB
 RED_LED     EQU .2 ; RB.5
 AMBER_LED   EQU .3 ; RB.6
 GREEN_LED   EQU .4 ; RB.7
 
 BUTTONPORT  EQU PORTB
 BUTTONDIR   EQU TRISB
 BUTTONPU    EQU WPUB
 ENTER       EQU .5 ; RB.4 (Input, internal pullup)
 LEFT        EQU .6 ; RB.2 (Input, internal pullup)
 RIGHT       EQU .7 ; RB.3 (Input, internal pullup)
 
 ; LCD 
 #define     LCDMASK     0x0F
 #define     STRH        0x00
 #define     STRL        0x20
 
 ; Data Memory
     cblock 0x20
     current_menu:1, menu_selected:1, menu_pointer:2, lcd_pointer:2, distance:2,
     menu_max:1, menu_min:1, upper_tolerance:2, lower_tolerance:2,
     upper_passband:2
     endc
     cblock 0x30
         lower_passband:2, X:2, Y:2, BCD:4 ,result:5, count:1
     endc
     cblock  0x40
         temp:1,usresult:1,ascii:6, current_value_pointer:1
     endc
     cblock      0x70
         deltime:1, cmdbyte:1
     endc
 
 ; Macros
 ; Moves l into f
 movlf   MACRO   ell,eff
         banksel eff
         movlw   ell
         movwf   eff
         ENDM
 
 ; Moves 16bit l into 16bit f
 movlf16 MACRO   ell,eff
         banksel eff
         movlw   low ell
         movwf   eff
         movlw   high ell
         movwf   eff+1
         ENDM
 
 ; Moves 8bit f1 into 8bit f2
 movff   MACRO   f1,f2
         banksel f1
         movf    f1,W
         banksel f2
         movwf   f2
         ENDM
 
 ; Moves 16bit f1 into 16bit f2
 movff16 MACRO   f1,f2
         banksel f1
         movf    f1,W
         banksel f2
         movwf   f2
         banksel f1+1
         movf    f1+1,W
         banksel f2
         movwf   f2+1
         ENDM
 
 ; Selects and clears f1
 clear   MACRO   f1
         banksel f1
         clrf    f1
         ENDM
 
 ; Moves f into FSRn
 movfi   MACRO   f1,fsrn
         banksel f1
         movf    f1,W
         movwi   0[fsrn]
         ENDM
 
 ; Loads fsrn and fsrn+1 into a 16-bit file
 movif16 MACRO   fsrn,f1
         moviw   0[fsrn]
         banksel f1
         movwf   f1
         moviw   1[fsrn]
         movwf   f1+1
         ENDM
 
 ; Moves l into fsrn
 movli   MACRO   ell,fsrn
         movlw   ell
         movwi   0[fsrn]
         ENDM
 
 ; Moves 16-bit l into fsrn and fsrn+1
 movli16 MACRO   ell,fsrn
         movlw   low ell
         movwi   0[fsrn]
         movlw   high ell
         movwi   1[fsrn]
         ENDM
 
 ; Moves 0[FSR0] and 1[FSR0] into f1 and f1+1
 moviw16 MACRO   fsrn,f1
         moviw   0[fsrn]
         banksel f1
         movwf   f1
 
         moviw   1[fsrn]
         movwf   f1+1
         ENDM
 
 ; Returns the lower nibble of a file as ASCII character using lookup table
 get_ascii_ln    MACRO   f1,f2
                 banksel f1
                 movf    f1,W
                 andlw   0x0F
                 call    get_ascii
                 banksel f2
                 movwf   f2
                 ENDM
 
 ; Returns the upper nibble of a file as ASCII chaarcter using lookup table
 get_ascii_un    MACRO   f1,f2
                 banksel f1
                 swapf   f1,W
                 andlw   0x0F
                 call    get_ascii
                 banksel f2
                 movwf   f2
                 ENDM
 
 ; Subtract l from f, place in w
 subfl_to_w      MACRO   l1,f1
                 banksel f1
                 movlw   l1
                 subwf   f1,w
                 ENDM
 
 ;*******************************************************************************
 ; Program
 ;*******************************************************************************
 
     org  0x00
     goto setup
     org 0x04
 ISR
 ;****************************
 ;   Strings
 ;****************************
 start_string              dt    "START MENU\0"
 scan_string               dt    "SCAN\0"
 testing_string            dt    "TESTING\0"
 upper_passband_string     dt    "UPPER PSBND\0"
 lower_passband_string     dt    "LOWER PSBND\0"
 upper_tolerance_string    dt    "UPPER TOL\0"
 lower_tolerance_string    dt    "LOWER TOL\0"
 pass_string               dt    "PASS: \0"
 fail_string               dt    "FAIL: \0"
 timeout_string            dt    "USOUND TIMEOUT\0"
 mm_string                 dt    " MM\0"
 add_100_string            dt    "ADD 100\0"
 add_10_string             dt    "ADD 10\0"
 add_1_string              dt    "ADD 1\0"
 sub_100_string            dt    "SUB 100\0"
 sub_10_string             dt    "SUB 10\0"
 sub_1_string              dt    "SUB 1\0"
 
 ;****************************
 ;   Setup
 ;
 ; Contains code for initialisation of PIC and ultrasound
 ; Only ever runs once, when PIC started up
 ;
 ; Initialises the system clock, pinout, LCD and waits 5ms to ensure Polaroid
 ; 6500 intialised.
 ;****************************
 setup
         ; use 4Mhz fosc
         movlf   0xEB,OSCCON
 
         movlf   0xF0,TMR0
         bsf     INTCON,TMR0IE ; Set TMR0 to count 5ms w/ pre of 4 and use Finst
         movlf   0x01,OPTION_REG
 
         ; Make sure no PORTB pins are analogue pins
         clear   ANSELB
 
         ; Initialise pin direction
         clear   TRISB           ;\ Set buttons as inputs
         bsf     TRISB,LEFT      ;|
         bsf     TRISB,RIGHT     ;|
         bsf     TRISB,ENTER     ;/
         bcf     TRISB,GREEN_LED ;\ Set LED pins as outputs
         bcf     TRISB,AMBER_LED ;|
         bcf     TRISB,RED_LED   ;/ 
         bcf     TRISB,INIT      ; Set INIT as output
         bsf     TRISB,ECHO      ; Set ECHO as input
 
         ; Switch CCP1 from RB3 to RB0
         banksel APFCON0
         bsf     APFCON0,CCP1SEL
         clear   WPUB            ;\Set pullups for buttons, will use
         bsf     WPUB,LEFT      ;| -ive edge triggering
         bsf     WPUB,RIGHT     ;|
         bsf     WPUB,ENTER     ;/
         bsf     WPUB,ECHO
         ; Initialise the LCD
         call    lcd_setup
 
         ; Enable -ive edge interrupts on buttons
         banksel IOCBN
         bsf     IOCBN,LEFT
         bsf     IOCBN,RIGHT
         bsf     IOCBN,ENTER
 
         ; Wait 5ms so that Polaroid 6500 is definitely initialised
         btfss   INTCON,TMR0IF
         goto    $-1
 
         ; Disable TMR0 and clear IF
         bcf     INTCON,TMR0IE 
         bcf     INTCON,TMR0IF
         clear   OPTION_REG
 
         ; Initialise variables
         movlf16   TOLERANCE_MIN,lower_tolerance
         movlf16   TOLERANCE_MAX, upper_tolerance
         movlf16   PASSBAND_MIN, lower_passband
         movlf16   PASSBAND_MAX, upper_passband
 
         ; Initialise the menu
         goto    set_start
 
 ;****************************
 ;   Main
 ;
 ; Program loop
 ; Default state is sleep, woken when a button is pressed
 ;
 ; Will then move to perform action appropriate for the button pressed
 ;
 ; When that action is finished, PIC moves into update_lcd,
 ;****************************
 main
         bsf INTCON,IOCIE
         sleep             ; Wait for input
 
         ; TMR0 sometimes overflows, not sure why yet but this stops it from
         ; waking the PIC up
         btfsc   INTCON,TMR0IF
         goto    TMR0_reset
 
         ; Disable IOC pins
         clear   PIR1
         bcf     INTCON,IOCIE
 
         banksel IOCBF
         btfsc   IOCBF,LEFT
         goto    cycle_left
 
         btfsc   IOCBF,RIGHT
         goto    cycle_right
 
         btfsc   IOCBF,ENTER
         goto    goto_menu
 
 TMR0_reset
         bcf     INTCON,TMR0IF
         goto    main
 
 ; Jump to location stored in menu_pointer
 goto_menu
     banksel menu_pointer
     movff    menu_pointer+1,PCLATH
     bsf      PCLATH,7
     movff    menu_pointer,PCL
 
 ; Cycles "left" through menu, error checks to make sure never go above max limit
 cycle_left
     banksel menu_selected
     incf menu_selected,F
     call error_check_upper
     goto update_lcd
 
 ; Cycles "right" thru menu, error checks to make sure never go above max limit
 cycle_right
     banksel menu_selected
     decf menu_selected,F
     call error_check_lower
     goto update_lcd
 
 ; Jump to location stored in lcd_pointer
 update_lcd
         movff   lcd_pointer+1,PCLATH
         movff   lcd_pointer,PCL
 
 ; Get ready to jump back to main
 fin
         clear IOCBF
         goto main
 
 ; Check if menu_option is less than minimum value for current menu, if so then
 ; copies minimum value into menu_option
 error_check_lower
     banksel menu_min
     movf   menu_min,W
     subwf  menu_selected,W
     movf   menu_max,W
     btfss  STATUS,C
     movwf  menu_selected
     return
 
 ; Check if menu_option is grater than maximum value for current menu, if so then
 ; copies maximum value into menu_option
 error_check_upper
     banksel menu_max
     movf   menu_max,W
     addlw  0x01
     subwf  menu_selected,W
     movf   menu_min,W
     btfsc  STATUS,C
     movwf  menu_selected
     return
 
 ; Jump to here from goto_menu when in start menu and enter button pushed. Branch
 ; to option currently selected
 start_menu
     movf menu_selected,W
     brw
     goto    software_error  ; W = 0
     goto    software_error  ; W = 1
     goto    perform_scan    ; W = 2
     goto set_upper_passband ; W = 3
     goto set_lower_passband ; W = 4
     goto set_upper_tolerance; W = 5
     goto set_lower_tolerance; W = 6
 
 ; Jump to here from goto_menu when in upper passband menu and enter button
 ; pushed. Branch to option currently selected
 upper_passband_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ; error()
     goto  add_100 ; add 100 to value
     goto  add_10  ; add 10 from value
     goto  add_1   ; add 1 from value
     goto  sub_100 ; subtract 100 from value
     goto  sub_10  ; subtract 10 from value
     goto  sub_1   ; subtract 1 from value
     goto  set_start
 
 ; Jump to here from goto_menu when in lower passband menu and enter button
 ; pushed. Branch to option currently selected
 lower_passband_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ; error()
     goto  add_100 ; add 100 to value
     goto  add_10  ; add 10 from value
     goto  add_1   ; add 1 from value
     goto  sub_100 ; subtract 100 from value
     goto  sub_10  ; subtract 10 from value
     goto  sub_1   ; subtract 1 from value
     goto  set_start
 
 ; Jump to here from goto_menu when in upper tolerance menu and enter button
 ; pushed. Branch to option currently selected
 upper_tolerance_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ; error()
     goto  add_100 ; add 100 to value
     goto  add_10  ; add 10 from value
     goto  add_1   ; add 1 from value
     goto  sub_100 ; subtract 100 from value
     goto  sub_10  ; subtract 10 from value
     goto  sub_1   ; subtract 1 from value
     goto  set_start
 
 ; Jump to here from goto_menu when in lower tolerance menu and enter button
 ; pushed. Branch to option currently selected
 lower_tolerance_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ; error()
     goto  add_100 ; add 100 to value
     goto  add_10  ; add 10 from value
     goto  add_1   ; add 1 from value
     goto  sub_100 ; subtract 100 from value
     goto  sub_10  ; subtract 10 from value
     goto  sub_1   ; subtract 1 from value
     goto  set_start
 
 ; Jump to here from goto_menu when enter button pushed and "add 100" selected
 ; Adds 100 the value located at the address stored in current_value_pointer, then error
 ; checks to make sure it never goes above the absolute limit
 add_100
     movff16 current_value_pointer,FSR0
 
     ; Add 100 to lower byte of current_value
     MOVIW   0[FSR0] 
     addlw  .100 ; 0x64
     MOVWI  0[FSR0]
 
     ; If there was an overflow, add 1 to upper byte of current_value
     MOVIW  1[FSR0]
     btfsc  STATUS,C
     addlw  .1
     MOVWI  1[FSR0]
 
 
     call   add_error_check
     btfsc  WREG,FAIL
     call   set_max_value
 
     goto   update_lcd ; write the new distance to the lcd
 
 ; Jump to here from goto_menu when enter button pushed and "add 10" selected
 ; Adds 10 to the value located at the address stored in current_value_pointer, then error
 ; checks to make sure it never goes above the absolute maximum limit
 add_10
     movff16 current_value_pointer,FSR0
 
     ; Add 10 to lower byte of current_value
     MOVIW  0[FSR0] ; move distance into w
     addlw  .10 ; 0x0A
     MOVWI  0[FSR0]
 
     ; If there was an overflow, add 1 to upper byte of current_value
     MOVIW  1[FSR0]
     btfsc  STATUS,C
     addlw  .1
     MOVWI  1[FSR0]
 
     ; Check if result > MAX_DIST, if so set current value to MAX_DIST
     call   add_error_check; error check: within limit?
     btfsc  WREG,FAIL         ; if without limit, set to max value
     call   set_max_value
 
     goto   update_lcd ; write the new distance to the lcd
 
 ; Jump to here from goto_menu when enter button pushed and "add 1" selected
 ; Adds 1 to the value located at the address stored in current_value_pointer, then
 ; error checks to make sure it never goes above the absolute maximum limit
 add_1
     movff16 current_value_pointer,FSR0
 
     ; Add 1 to lower byte of current_value
     MOVIW  0[FSR0] ; move distance into w
     addlw  .1 ; 0x01
     MOVWI  0[FSR0]
 
     ; If there was an overflow, add 1 to upper byte of current_value
     MOVIW  1[FSR0]
     btfsc  STATUS,C
     addlw  .1
     MOVWI  1[FSR0]
 
     ; Check if result > MAX_DIST, if so set current value to MAX_DIST
     call   add_error_check; error check: within limit?
     btfsc  WREG,FAIL         ; if without limit, set to max value
     call   set_max_value
 
     goto   update_lcd ; write the new distance to the lcd
 
 ; Jump to here from goto_menu when enter button pushed and "sub 100" selected
 ; Subtracts 100 from the value located at the address stored in 
 ; current_value_pointer, then error checks to make sure it never goes below
 ; the absolute minimum limit
 sub_100
     movff16 current_value_pointer,FSR0
     banksel temp
 
     ; Move lower byte of current_value to temp, then subtract 100 from it and
     ; move back into current_value
     moviw   0[FSR0]
     movwf   temp
     movlw   .100 ; 0x64
     subwf   temp,W
     movwi   0[FSR0]
 
     ; If lower byte underflowed, subtract 1 from upper byte of current_value
     moviw   1[FSR0]
     movwf   temp
     movlw   .1
     btfss  STATUS,C
     subwf  temp,F
     movf    temp,W
     movwi   1[FSR0]
 
     ; Check if result < MIN_DIST, if so set current value to MIN_DIST
     call   sub_error_check ; error check: within limit?
     btfsc  WREG,FAIL         ; if without limit, set to max value
     call   set_min_value
 
     goto   update_lcd ; write the new distance to the lcd
 
 ; Jump to here from goto_menu when enter button pushed and "sub 10" selected
 ; Subtracts 10 from the value located at the address stored in
 ; current_value_pointer, then error checks to make sure it never goes below
 ; the absolute minimum limit
 sub_10
     movff16 current_value_pointer,FSR0
     banksel temp
 
     ; Move lower byte of current_value to temp, then subtract 10 from it and
     ; move back into current_value
     moviw   0[FSR0]
     movwf   temp
     movlw   .10 ; 0x64
     subwf   temp,W
     movwi   0[FSR0]
 
     ; If lower byte underflowed, subtract 1 from upper byte of current_value
     moviw   1[FSR0]
     movwf   temp
     movlw   .1
     btfss  STATUS,C
     subwf  temp,F
     movf    temp,W
     movwi   1[FSR0]
 
     ; Check if result < MIN_DIST, if so set current value to MIN_DIST
     call   sub_error_check ; error check: within limit?
     btfsc  WREG,FAIL         ; if without limit, set to max value
     call   set_min_value
 
     goto   update_lcd ; write the new distance to the lcd
 
 ; Jump to here from goto_menu when enter button pushed and "sub 1" selected
 ; Subtracts 1 from the value located at the address stored in
 ; current_value_pointer, then error checks to make sure it never goes below
 ; the absolute minimum limit
 sub_1
     movff16 current_value_pointer,FSR0
     banksel temp
 
     ; Move lower byte of current_value to temp, then subtract 1 from it and
     ; move back into current_value
     moviw   0[FSR0]
     movwf   temp
     movlw   .1 ; 0x64
     subwf   temp,W
     movwi   0[FSR0]
 
     ; If lower byte underflowed, subtract 1 from upper byte of current_value
     moviw   1[FSR0]
     movwf   temp
     movlw   .1
     btfss  STATUS,C
     subwf  temp,F
     movf    temp,W
     movwi   1[FSR0]
 
     call   sub_error_check ; error check: within limit?
     btfsc  WREG,FAIL         ; if without limit, set to max value
     call   set_min_value
 
     ; Check if result < MIN_DIST, if so set current value to MIN_DIST
     goto   update_lcd ; write the new distance to the lcd
 
 set_max_value
     movli16 DIST_MAX,FSR0
     return
 
 set_min_value
     movli16 DIST_MIN,FSR0
     return
 
 ; See if current_value > DIST_MAX by seeing if (current_value - DIST_MAX)
 ; underflows and sets the carry bit
 add_error_check
     MOVIW   0[FSR0]
     movwf   X
     MOVIW   1[FSR0]
     movwf   X+1
 
     movlf16 DIST_MAX,Y
 
     call    subt16x16
 
     btfss   STATUS,C
     goto    add_error_check_fail
 
     goto    add_error_check_pass
 add_error_check_fail
     retlw FAIL
 add_error_check_pass
     retlw PASS
 
 ; See if current_value < DIST_MIN by seeing if (DIST_MIN - current_value)
 ; underflows and sets the carry bit
 sub_error_check
     MOVIW 0[FSR0]
     movwf Y
     MOVIW 1[FSR0]
     movwf Y+1
 
     movlf16 DIST_MIN,X
 
     call  subt16x16
 
     btfss STATUS,C
     goto sub_error_check_fail
 
     goto sub_error_check_pass
 sub_error_check_fail
     retlw FAIL
 sub_error_check_pass
     retlw PASS
 
 ; Call scan, get result (ie passed, within tolerance, failed, or timeout
 ; Then choose goto to print appropriate string
 perform_scan
     call    scan
 
     btfsc   usresult,PASS
     goto    print_pass_string
 
     btfsc   usresult,TOLERANCE
     goto    print_tolerance_string
 
     btfsc   usresult,FAIL
     goto    print_fail_string
 
     btfsc   usresult,TIMEOUT
     goto    print_timeout_string
 
 ; Prepare for Start Menu to become current menu
 set_start
     ; Reset menu_selected
     movlf   START_MENU_MIN,menu_selected
 
     ; Set minimum possible menu
     movlf   START_MENU_MIN,menu_min
 
     ; Set maximum possible menu
     movlf   START_MENU_MAX,menu_max
 
     ; Copy location of start menu to menu_pointer
     movlf16 start_menu,menu_pointer
 
     ; Copy location of info for printing lcd for the start meny
     movlf16 update_lcd_start_menu,lcd_pointer
 
     goto  update_lcd
 
 ; Prepare for Upper Tolerance Menu to become current menu
 set_upper_tolerance
     movlf   UPPER_TOLERANCE_MENU_MIN,menu_selected
     movlf   UPPER_TOLERANCE_MENU_MIN,menu_min
     movlf   UPPER_TOLERANCE_MENU_MAX,menu_max
     movlf16 upper_tolerance,current_value_pointer
     movlf16 upper_tolerance_menu,menu_pointer
     movlf16 update_lcd_upper_tolerance_menu,lcd_pointer
     goto  update_lcd
 
 ; Prepare for Lower Tolerance Menu to become current menu
 set_lower_tolerance
     movlf   LOWER_TOLERANCE_MENU_MIN,menu_selected
     movlf   LOWER_TOLERANCE_MENU_MIN,menu_min
     movlf   LOWER_TOLERANCE_MENU_MAX,menu_max
     movlf   lower_tolerance,current_value_pointer
     movlf16 lower_tolerance_menu,menu_pointer
     movlf16 update_lcd_lower_tolerance_menu,lcd_pointer
     goto  update_lcd
 
 ; Prepare for Upper Passband Menu to become current menu
 set_upper_passband
     movlf   UPPER_PASSBAND_MENU_MIN,menu_selected
     movlf   UPPER_PASSBAND_MENU_MIN,menu_min
     movlf   UPPER_PASSBAND_MENU_MAX,menu_max
     movlf16   upper_passband,current_value_pointer
     movlf16 upper_passband_menu,menu_pointer
     movlf16 update_lcd_upper_passband_menu,lcd_pointer
     goto  update_lcd
 
 ; Prepare for Lower Passband to become current menu
 set_lower_passband
     movlf   LOWER_PASSBAND_MENU_MIN,menu_selected
     movlf   LOWER_PASSBAND_MENU_MIN,menu_min
     movlf   LOWER_PASSBAND_MENU_MAX,menu_max
     movlf16   lower_passband,current_value_pointer
     movlf16 lower_passband_menu,menu_pointer
     movlf16 update_lcd_lower_passband_menu,lcd_pointer
     goto  update_lcd
 
 ; Jump to the appropriate print function for Start Menu
 update_lcd_start_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ;error
     goto print_scan_string
     goto print_upper_passband_string
     goto print_lower_passband_string
     goto print_upper_tolerance_string
     goto print_lower_tolerance_string
 
 ; Jump to the appropriate print function for Upper Passband Menu
 update_lcd_upper_passband_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ;error
     goto print_add_100_string
     goto print_add_10_string
     goto print_add_1_string
     goto print_sub_100_string
     goto print_sub_10_string
     goto print_sub_1_string
     goto print_start_string
 
 ; Jump to the appropriate print function for Lower Passband Menu
 update_lcd_lower_passband_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ;error
     goto print_add_100_string
     goto print_add_10_string
     goto print_add_1_string
     goto print_sub_100_string
     goto print_sub_10_string
     goto print_sub_1_string
     goto print_start_string
 
 ; Jump to the appropriate print function for Upper Tolerance Menu
 update_lcd_upper_tolerance_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ;error
     goto print_add_100_string
     goto print_add_10_string
     goto print_add_1_string
     goto print_sub_100_string
     goto print_sub_10_string
     goto print_sub_1_string
     goto print_start_string
 
 ; Jump to the appropriate print function for Lower Tolerance Menu
 update_lcd_lower_tolerance_menu
     movf menu_selected,W
     brw
     NOP ;error
     NOP ;error
     goto print_add_100_string
     goto print_add_10_string
     goto print_add_1_string
     goto print_sub_100_string
     goto print_sub_10_string
     goto print_sub_1_string
     goto print_start_string
 
 ; Print "START" to top line of the LCD
 print_start_string
     call    lcd_clear ; clear the lcd
     call    set_lcd_top_line ; set lcd to write on top line
     movlf16 start_string,FSR0
     call    lcd_print      ; write scan_string to lcd_print
     goto    fin
 
 ; Print "SCAN" to top line of the LCD
 print_scan_string
     call    lcd_clear ; clear the lcd
     call    set_lcd_top_line ; set lcd to write on top line
     movlf16 scan_string,FSR0
     call    lcd_print      ; write scan_string to lcd_print
     goto    fin
 
 ; Print "SCAN" to top line of the LCD, and "PASS: XXXX CM" to bottom line of lcd
 ; where XXXX is the four-digit result from scan. Sets GREEN_LED, and clears
 ; AMBER_LED and RED_LED
 print_pass_string
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 scan_string, FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf16 pass_string, FSR0
     call    lcd_print
     movlf16 ascii, FSR0
     call    lcd_print
     movlf16 mm_string, FSR0
     call    lcd_print
 
     banksel LEDPORT
     bsf     LEDPORT, GREEN_LED
     bcf     LEDPORT, AMBER_LED
     bcf     LEDPORT, RED_LED
 
     clear usresult
 
     goto    fin
 
 ; Print "SCAN" to top line of the LCD, and "FAIL: XXXX CM" to bottom line of lcd
 ; where XXXX is the four-digit result from scan. Sets AMBER_LED, and clears
 ; GREEN_LED and RED_LED
 print_tolerance_string
     call    hex16_2_ascii
 
     call  lcd_clear ; clear the lcd
     call  set_lcd_top_line ; set lcd to write on top line
     movlf16 scan_string, FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf16 fail_string, FSR0
     call    lcd_print
     movlf16 ascii, FSR0
     call    lcd_print
     movlf16 mm_string, FSR0
     call    lcd_print
 
     banksel LEDPORT
     bcf     LEDPORT, GREEN_LED
     bsf     LEDPORT, AMBER_LED
     bcf     LEDPORT, RED_LED
 
     clear usresult
 
     goto    fin
 
 ; Print "SCAN" to top line of the LCD, and "FAIL: XXXX CM" to bottom line of lcd
 ; where XXXX is the four-digit result from scan. Sets RED_LED, and clears
 ; GREEN_LED and AMBER_LED
 print_fail_string
     ; value should already be in X
     call    hex16_2_ascii
 
     call  lcd_clear ; clear the lcd
     call  set_lcd_top_line ; set lcd to write on top line
     movlf16 scan_string, FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf16 fail_string, FSR0
     call    lcd_print
     movlf16 ascii, FSR0
     call    lcd_print
     movlf16 mm_string, FSR0
     call    lcd_print
 
     banksel LEDPORT
     bcf     LEDPORT, GREEN_LED
     bcf     LEDPORT, AMBER_LED
     bsf     LEDPORT, RED_LED
 
     clear usresult
 
     goto    fin
 
 ; Print "SCAN" to top line of the LCD, and "TIMEOUT" to bottom line of lcd
 ; Clears all three LEDs
 print_timeout_string
     call    lcd_clear ; clear the lcd
     call    set_lcd_top_line ; set lcd to write on top line
     movlf16 scan_string, FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf16 timeout_string, FSR0
     call    lcd_print
 
     banksel LEDPORT
     bCf     LEDPORT, GREEN_LED
     bcf     LEDPORT, AMBER_LED
     bsf     LEDPORT, RED_LED
 
     bcf LEDPORT, GREEN_LED
     bcf LEDPORT, AMBER_LED
     bcf LEDPORT, RED_LED
 
     clear usresult
 
     goto    fin
 
 ; Print "UPPER PASSBAND" to top line of the LCD
 print_upper_passband_string
     call  lcd_clear
     call  set_lcd_top_line
     movlf   upper_passband_string,FSR0
     call  lcd_print
     goto    fin
 
 ; Print "LOWER PASSBAND" to top line of the LCD
 print_lower_passband_string
     call  lcd_clear
     call  set_lcd_top_line
     movlf16   lower_passband_string,FSR0
     call  lcd_print
     goto  fin
 
 ; Print "UPPER TOLERANCE" to top line of the LCD
 print_upper_tolerance_string
     call  lcd_clear
     call  set_lcd_top_line
     movlf16  upper_tolerance_string,FSR0
     call  lcd_print
     goto  fin
 
 ; Print "LOWER TOLERANCE" to top line of the LCD
 print_lower_tolerance_string
     call  lcd_clear
     call  set_lcd_top_line
     movlf16   lower_tolerance_string,FSR0
     call  lcd_print
     goto  fin
 
 ; Print "ADD 100" to top line of the LCD, and "XXXX MM" to bottom line of lcd
 ; where XXXX is the four-digit value of (current_value + 100)
 print_add_100_string
     movff16 current_value_pointer,FSR0
     moviw16 FSR0,X
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 add_100_string,FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf   ascii,FSR0
     bcf     FSR0H,7
     call    lcd_print
     movlf   mm_string,FSR0
     call    lcd_print
 
     goto    fin
 
 ; Print "ADD 10" to top line of the LCD, and "XXXX MM" to bottom line of lcd
 ; where XXXX is the four-digit resut of (current_value + 10)
 print_add_10_string
     movff16 current_value_pointer,FSR0
     moviw16 FSR0,X
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 add_10_string,FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf   ascii,FSR0
     ; FSR0H.7 indicates location is in program memory, clear it just in case
     ; it hasn't been cleared, since ascii is in data memory
     bcf     FSR0H,7
     call    lcd_print
     movlf   mm_string,FSR0
     call    lcd_print
 
     goto    fin
 
 ; Print "ADD 1" to top line of the LCD, and "XXXX MM" to bottom line of lcd
 ; where XXXX is the four-digit value of (current_value + 1)
 print_add_1_string
     movff16 current_value_pointer,FSR0
     moviw16 FSR0,X
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 add_1_string,FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf   ascii,FSR0
     ; FSR0H.7 indicates location is in program memory, clear it just in case
     ; it hasn't been cleared, since ascii is in data memory
     bcf     FSR0H,7
     call    lcd_print
     movlf   mm_string,FSR0
     call    lcd_print
 
     goto    fin
 
 ; Print "SUB 100" to top line of the LCD, and "XXXX MM" to bottom line of lcd
 ; where XXXX is the four-digit value of (current_value - 1)
 print_sub_100_string
     movff16 current_value_pointer,FSR0
     moviw16 FSR0,X
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 sub_100_string,FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf   ascii,FSR0
     ; FSR0H.7 indicates location is in program memory, clear it just in case
     ; it hasn't been cleared, since ascii is in data memory
     bcf     FSR0H,7
     call    lcd_print
     movlf   mm_string,FSR0
     call    lcd_print
 
     goto    fin
 
 ; Print "SUB 10" to top line of the LCD, and "XXXX MM" to bottom line of lcd
 ; where XXXX is the four-digit value of (current_value - 10)
 print_sub_10_string
     movff16 current_value_pointer,FSR0
     moviw16 FSR0,X
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 sub_10_string,FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf   ascii,FSR0
     ; FSR0H.7 indicates location is in program memory, clear it just in case
     ; it hasn't been cleared, since ascii is in data memory
     bcf     FSR0H,7
     call    lcd_print
     movlf   mm_string,FSR0
     call    lcd_print
 
     goto    fin
 
 ; Print "SUB 1" to top line of the LCD, and "XXXX MM" to bottom line of lcd
 ; where XXXX is the four-digit value of (current_value - 1)
 print_sub_1_string
     movff16 current_value_pointer,FSR0
     moviw16 FSR0,X
     call    hex16_2_ascii
 
     call    lcd_clear
     call    set_lcd_top_line
     movlf16 sub_1_string,FSR0
     call    lcd_print
 
     call    set_lcd_bottom_line
     movlf   ascii,FSR0
     ; FSR0H.7 indicates location is in program memory, clear it just in case
     ; it hasn't been cleared, since ascii is in data memory
     bcf     FSR0H,7
     call    lcd_print
     movlf   mm_string,FSR0
     call    lcd_print
 
     goto    fin
 
 ;*******************************************************************************
 ; SCAN
 ;
 ; Gets a result from the Polaroid 6500, in units of microseconds
 ; Processes it, and then returns a value indicating whether passed, failed,
 ; within tolerance or timed out
 ;
 ; Starts with a 16-bit result from the Polaroid 6500 in microseconds
 ;
 ; The result is processed as follows:
 ;
 ; 1. (((result from Polaroid 6500) - PULSEGEN) * MM_PER_US)/1024) - LATENCY
 ; 2. The above is then compared to passband, if within the condition is PASS
 ; 3. Then compared to tolerance, if within condition is TOLERANCE
 ; 4. Must be outside passband or tolerance, therefore condition is FAIL
 ;
 ; If the PIC doesn't detect a reponse from ECHO before TMR1 overflow, then
 ; condition is TIMEOUT
 ;*******************************************************************************
 scan
                     call    ultrasound
  
                     banksel usresult
                     btfsc   usresult,PASS
                     goto    scan_pass
                     btfsc   usresult,FAIL
                     goto    scan_timeout
 scan_pass           clear   usresult
                     ; Recover result from CCPR1
                     movff   CCPR1L,X
                     movff   CCPR1H,X+1
                     ;movlf16   0x1D60,X
                     ; subtract ultrasound clock generation delay
                     movlf16 PULSEGEN,Y
                     call    subt16x16
                     ; multiply by (mm per instruction clock cycle)*1024
                     movff   result,X
                     movff   result+1,X+1
                     movlf16 MM_PER_US,Y
                     call    mult16x16       ;
                     ; divide by 1024 to get (mm per instruction clock cycle)
                     movlf   .10,Y
                     call    div40bin
                     ; subtract 20cm from result, fiddle factor
                     movff16 result,X
                     movlf16 LATENCY,Y
                     call    subt16x16
                     ; check if the result is less than the lower limit
                     movff16 result,X ; expect a 16 bit result so this is ok
                     movff16 lower_passband,Y
                     call    subt16x16
                     btfss   STATUS,C        ;|\ If Z set, then either X = Y, or
                     goto    below_passband     ;// X > Y If Z clear, then X < Y
                     movwf   temp            ;
                     ; check if result is greater than the upper limit
                     movff16 upper_passband,Y
                     call    subt16x16
                     btfsc   STATUS,C
                     goto    above_passband
                     ; if you get here then hooray, the value is in the passband
                     bsf     usresult,PASS
                     goto    scan_end
 
 below_passband
                     ; test if within tolerance
                     movff16 lower_tolerance,Y
                     call    subt16x16
                     btfss   STATUS,C
                     goto    scan_fail
                     ; if you get here then the value is within tolerance
                     bsf     usresult, TOLERANCE
                     goto    scan_end
 
 above_passband
                     movff16 upper_tolerance,Y
                     call    subt16x16
                     btfsc   STATUS,C
                     goto    scan_fail
                     ; test if within tolerance
                     ; if you get here then within tolerance
                     bsf     usresult, TOLERANCE
                     goto scan_end
 
 scan_fail
                     bsf     usresult, FAIL
                     goto    scan_end
 scan_timeout
                     clear   usresult
                     bsf     usresult,TIMEOUT
                     goto    scan_end
 scan_end 
                     return
 
 ; Communicate with Polaroid 6500 and get a result in terms of time
 ultrasound
                 ; Clear GIE because we don't want to use the ISR. Enable PEIE so
                 ; that Interrupt Flags can operate
                 clrf    INTCON
                 bcf     INTCON,GIE
                 bsf     INTCON,PEIE
                 ; Set TMR1IE and CCP1IE because we will use these modules
                 banksel PIE1
                 bsf     PIE1,TMR1IE
                 bsf     PIE1,CCP1IE
                 ; Use instruction clock as Timer 1 input, 1MHz -> 1us per
                 ; increment
                 movlf   0x05,T1CON
                 ; Set INIT to begin scan
                 banksel PORTB
                 bsf     PORTB,INIT
                 ; Clean TMR1 out just in case it isn't empty
                 banksel TMR1H
                 clrf    TMR1H
                 clrf    TMR1L
                 bcf     INTCON,TMR0IF
                 ; Reset TMR0 configuration
                 banksel OPTION_REG
                 bsf     OPTION_REG,TMR0CS
                 movlf   0x05,CCP1CON
                 ; Poll IFs in infinite loop until we have a result or timeout
                 banksel PIR1
 wait_loop
                 ; If TMR1IF set, then overflow ergo ultrasound must not be
                 ; responding
                 btfsc   PIR1,TMR1IF
                 goto    timeout
                 ; If CCP1IF set, then we must have got a result from the
                 ; Polaroid 6500
                 btfsc   PIR1,CCP1IF
                 goto    success
                 ; If neither IF's are set, loop back until one is
                 goto    wait_loop
 timeout
                 banksel usresult
                 bsf     usresult,FAIL
                 goto    got_result
 success
                 banksel usresult
                 bsf     usresult,PASS
                 goto    got_result
 
 got_result
                 ; Clear INIT to stop the Polaroid 6500 repeating the test
                 banksel PORTB
                 bcf     PORTB,INIT
                 ; Switch peripheral interrupts off because we don't need them
                 ; right now
                 bcf     INTCON,PEIE
                 ; Disable CCP1, TMR1, and PIR1
                 clear   CCP1CON
                 clear   T1CON
                 clear   PIR1
                 ; Clear Interrupt Flags
                 clear   PIE1
                 ; Set TMR0 to overflow after 80ms elapsed
                 movlf   0x12,OSCCON
                 bcf     INTCON,TMR0IF
                 movlf   0x62,TMR0
                 ; Set TMR0 up with a prescaler of 4 and to use instruction clock
                 ; (1Mhz)
                 movlf   0x01,OPTION_REG
 ults_end
                 ; Wait until 80ms elapsed before moving on
                 btfss   INTCON,TMR0IF
                 goto    $-1
                 ; Disable TMR0
                 bcf     INTCON,TMR0IF
                 ; Reset TMR0 configuration
                 banksel OPTION_REG
                 bsf     OPTION_REG,TMR0CS
                 ; Reset System Clock to 4MHz
                 movlf   0xEB,OSCCON                                     
                 return
 
 ;*******************************************************************************
 ; MATHS FUNCTIONS
 ;
 ;******************************************************************************
 
 ; Multiply together 16-bit integer X, and 16-bit integer Y and place into 40-bit
 ; Result register
 mulxy   MACRO   byte,bit
         btfsc   byte,bit
         call    add_y
         rrf result+4,F
         rrf result+3,F
         rrf result+2,F
         rrf result+1,F
         rrf result,F
         ENDM
 mult16x16
         bcf     STATUS,C  ; Clear Carry bit before we use it
         banksel X
         clrf    result+4   ; \  Clear registers that hold result
         clrf    result+3   ;  |
         clrf    result+2   ;  |
         clrf    result+1   ;  | 
         clrf    result     ; /
         mulxy   X,0   ; \  Perform long multiplication on X and Y, make the
         mulxy   X,1   ;  |
         mulxy   X,2   ;  |
         mulxy   X,3   ;  |
         mulxy   X,4   ;  |
         mulxy   X,5   ;  |
         mulxy   X,6   ;  |
         mulxy   X,7   ;  |
         mulxy   X+1,0 ;  |
         mulxy   X+1,1 ;  |
         mulxy   X+1,2 ;  |
         mulxy   X+1,3 ;  |
         mulxy   X+1,4 ;  |
         mulxy   X+1,5 ;  |
         mulxy   X+1,6 ;  |
         mulxy   X+1,7 ; /
         return
 add_y
         banksel Y
         movf    Y,W          ;\ Add least significant byte of Y to 2nd most
         addwf   result+2,F   ;/ sig. byte of result, if overflow set carry bit
         movf    Y+1,W        ;\ Add most significant byte of Y and C to most
         addwfc  result+3,F   ;/ sig. byte of result, if overflow set carry bit
         btfss   STATUS,C     ;\ Carry bit set -> result+3 oflows into result+4
         goto    fin_y        ;| .'. add carry bit to result+4
         movlw   0x00         ;|
         addwfc  result+4     ;/
 fin_y   return
 
 ; Add 16-bit integer Y to 16-bit integer X, and place into result register
 add16x16
         movf   Y,W         ;\ Add lower byte of Y to lower byte of X
         addwf  X,W         ;|
         movwf  result      ;/
         movf   Y+1,W       ;\ Add upper byte of Y to upper byte of X
         addwfc X+1,W       ;|
         movwf  result+1    ;/
         movwf  0x00        ;\ Add carry to overflow
         addwfc result+2,F  ;/
         return       ;
 
 ; Subtract a 16-bit integer Y from 16-bit integer X, place into result register
 subt16x16
             banksel Y
             movf    Y,W
             subwf   X,W
             movwf   result
             ;
             movf    Y+1,W
             btfss   STATUS,C
             addlw   0x01
             subwf   X+1,W
             movwf   result+1
             ;
             return
 
 ; Divide 40-bit wide result register by 1024 by bit shifting 10 places
 div40bin
 div         bcf  STATUS,C    ;|
             rrf  result+4,f  ;|\ Shift right from MSreg to LSreg
             rrf  result+3,f  ;||
             rrf  result+2,f  ;||
             rrf  result+1,f  ;||
             rrf  result,f    ;|/
             decfsz  Y,f      ;| Decrement count, exit loop when zero
             goto    div      ;/
             return
 
 ;*******************************************************************************
 ; LCD FUNCTIONS
 ;*******************************************************************************
 
 lcd_setup
             clear   ANSELA
             clear   LCDDIR
             clear   LCDPORT
             ; Set WDT for 1s
             movlf   0x15,WDTCON
 
             sleep
 
             clear   WDTCON
 
             movlf   .10,deltime
 
             movlw       0x02         ;Set 4 bit mode
             call        enable
             call        delay
 
             movlw       0x02         ;Set 4 bit mode
             call        enable
             call        delay
 
             movlw       0x08         ;Set 2 line
             call        enable
             call        delay
 
             movlf       0x0F, cmdbyte ;Display on, cursor on, blinking on
             call        lcdcmd
 
             movlf       0x06, cmdbyte ;Entry mode set.
             call        lcdcmd
 
             return
 
 lcdbusy
             banksel LCDDIR
             bsf     LCDDIR,DB7
             clear   LCDPORT
             bsf     LCDPORT,RW
 
 lcdbsy1     bsf     LCDPORT,EN
             nop
             movf    LCDPORT,w
             bcf     LCDPORT,EN
             nop
 
             bsf     LCDPORT,EN
             nop
             bcf     LCDPORT,EN
             nop
 
             btfsc   WREG,DB7
             goto    lcdbsy1
 
             clear   LCDDIR
             return
 
 lcdcmd
             swapf       cmdbyte,w   ;\
             andlw       LCDMASK     ; | Output the LCD command stored in cmdbyte
             call        enable      ; |
 
             movf        cmdbyte,w   ; | Outputs high nibble then low nibble.
             andlw       LCDMASK     ; |
             call        enable      ; |
                                     ; |
             call        lcdbusy     ; |
                                     ; |
             return                  ;/
 
 set_lcd_top_line
             movlf       0x80,cmdbyte ; Move cursor to line 1
             call        lcdcmd
             return
 
 ; Move cursor to line 2
 set_lcd_bottom_line
 
             movlf       0xC0, cmdbyte
             call        lcdcmd
             return
 
 ; Clear screen
 lcd_clear
             movlf       0x01, cmdbyte
             call        lcdcmd
             return
 
 ; Turn screen off
 lcdoff
             movlf       0x08, cmdbyte
             call        lcdcmd
             return
 
 ; Turn Screen on
 lcdon
             movlf       0x0C, cmdbyte
             call        lcdcmd
             return
 
 ; Print string to LCD
 lcd_print
             moviw       0[FSR0]
             andlw       0xFF
             btfsc       STATUS,Z    ;
             return                  ;
             swapf       WREG,w
             andlw       LCDMASK     ; Move first string byte from FSR, if zero
             bsf         WREG,RS     ; (null), return as end of string reached.
             call        enable
 
             moviw       FSR0++      ; Send high then low nibble of character
             andlw       LCDMASK     ; incrementing FSR then call 'lcdbusy'.
             bsf         WREG,RS
             call        enable
 
             call        lcdbusy
 
             goto        lcd_print    ; Will loop until end of string reached.
 
 enable
             banksel     LCDPORT     ;\
             movwf       LCDPORT     ; | Loads nibble onto LCD data lines, sets
             bsf         LCDPORT,EN  ; | the enable line then clears the LCD
             nop                     ; | port.
             clrf        LCDPORT     ; |
             return                  ;/
 
 delay       movf        deltime,w   ;\
             goto        deljmp      ; | Should be a 10us delay each loop, uses
 delloop     nop                     ; | number stored in 'deltime' for iteration
             nop                     ; | count.
             nop                     ; |
             nop                     ; | First loop shorter due to instruction
             nop                     ; | times calling function.
             nop                     ; |
 deljmp      nop                     ; | 10us only valid for 4MHz clock.
             decfsz      WREG,w      ; |
             goto        delloop     ; |
             return                  ;/
 
 ;*******************************************************************************
 ; HEX16 TO ASCII
 ;
 ; Convert a 16-bit register X:2 into a 4 radix wide denary number, ascii:5
 ;*******************************************************************************
 hex16_2_ascii
             call hex16_2_bcd
             get_ascii_un    BCD+1, ascii
             get_ascii_ln    BCD+1, ascii+1
             get_ascii_un    BCD+2, ascii+2
             get_ascii_ln    BCD+2, ascii+3
             movlf           "\0",  ascii+4
 
             return
 
 hex16_2_bcd
     bcf STATUS, C
     movlf   .16, count
     clear   BCD
     clear   BCD+1
     clear   BCD+2
     clear   BCD+3
 hex16_2_bcd_loop
     banksel X
     rlf X,f
     rlf X+1,f
 
     banksel BCD
     rlf BCD+2,f
     rlf BCD+1,f
     rlf BCD,f
 
     banksel count
     decfsz  count,f
     goto    adjDEC
 
     return
 
 adjDEC
     movlf   BCD+2,FSR1
     call    adjBCD
 
     movlf   BCD+1,FSR1
     call    adjBCD
 
     movlf   BCD,FSR1
     call    adjBCD
 
     goto    hex16_2_bcd_loop
 
 adjBCD
     movf    INDF1,w
     addlw   0x03
     btfsc   WREG,3
     movwi   0[FSR1]
 
     moviw   0[FSR1]
     addlw   0x30
     btfsc   WREG,7
     movwi   0[FSR1]
 
     return
 
 ; get_ascii
 ; Returns the ASCII equivalent of a single BCD number
 ; Uses the least significant nibble
 get_ascii
     brw
     retlw   "0"
     retlw   "1"
     retlw   "2"
     retlw   "3"
     retlw   "4"
     retlw   "5"
     retlw   "6"
     retlw   "7"
     retlw   "8"
     retlw   "9"
 
 ;*******************************************************************************
 ; SOFTWARE ERROR
 ; You don't want to end up here
 ; Sets all three LEDs ON
 ;*******************************************************************************
 
 software_error
     BANKSEL TRISB
     bsf     TRISB,GREEN_LED
     bsf     TRISB,AMBER_LED
     bsf     TRISB,RED_LED
     goto    $
 
                 end