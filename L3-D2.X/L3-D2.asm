#include <p18f4620.inc>
#include <lcd18.inc>
list P=18F4620, F=INHX32, C=160, N=80, ST=OFF, MM=OFF, R=DEC

; ****************************************************************************
; Configuration Bits
; ****************************************************************************

		CONFIG OSC=HS, FCMEN=OFF, IESO=OFF
		CONFIG PWRT = OFF, BOREN = SBORDIS, BORV = 3
		CONFIG WDT = OFF, WDTPS = 32768
		CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF, CCP2MX = PORTC
		CONFIG STVREN = ON, LVP = OFF, XINST = OFF
		CONFIG DEBUG = OFF
		CONFIG CP0 = OFF, CP1 = OFF, CP2 = OFF, CP3 = OFF
		CONFIG CPB = OFF, CPD = OFF
		CONFIG WRT0 = OFF, WRT1 = OFF, WRT2 = OFF, WRT3 = OFF
		CONFIG WRTB = OFF, WRTC = OFF, WRTD = OFF
		CONFIG EBTR0 = OFF, EBTR1 = OFF, EBTR2 = OFF, EBTR3 = OFF
		CONFIG EBTRB = OFF

; ****************************************************************************
; CONSTANT DEFINES
; ****************************************************************************
#define     BreakBeam           PORTA, 5
#define     Photo1              PORTE, 0
#define     Photo2              PORTE, 1
#define     Photo3              PORTE, 2
#define		ArmSol              LATC, 5
#define     GripSol             LATC, 2
#define     Step1a              LATA, 0
#define     Step1b              LATA, 1
#define     Step2a              LATA, 2
#define     Step2b              LATA, 3
#define     StepDelayVal        0x3F                                                                    ; EDIT FOR STEP DELAY

key1		equ		d'0'
key2		equ		d'1'
key3		equ		d'2'
keyA		equ		d'3'
key4		equ		d'4'
key5		equ		d'5'
key6		equ		d'6'
keyB		equ		d'7'
key7		equ		d'8'
key8		equ		d'9'
key9		equ		d'10'
keyC		equ		d'11'
keyStar		equ		d'12'
key0		equ		d'13'
keyHash		equ		d'14'
keyD		equ		d'15'

STATUS_TEMP equ     0x00
W_TEMP      equ     0x01

; 0x10 to 0x29 for LCD

delayReg	equ		0x30

KEY			equ		0x50
KEY_Temp	equ		0x51
KEY_ISR     equ     0x52

InOperation equ     0x60
TrayEncoder equ     0x61
stepDelay1  equ     0x62
stepDelay2  equ     0x63
FL_count    equ     0x64

PhotoInput  equ     0x70

EEPROM_H    equ     0x80
EEPROM_L    equ     0x81
EEPROM_REG	equ		0x82
LED_Count	equ		0x83
SkipCount	equ		0x84

RTC_Second  equ		0x90
RTC_Minute  equ		0x91
RTC_Hour    equ		0x92
RTC_Day     equ		0x93
RTC_Date    equ		0x94
RTC_Month   equ		0x95
RTC_Year    equ		0x96
RTC_L       equ		0x97
RTC_H       equ		0x98


; ****************************************************************************
; MACROS
; ****************************************************************************

; Move a literal into F
movlf       macro		literal, reg
    movlw   literal
    movwf   reg
			endm

; Save/Restore Context in Subroutines
saveContext macro
    movff   STATUS, STATUS_TEMP     ; save STATUS first
    movwf   W_TEMP                  ; save W
			endm
restContext macro
    swapf   W_TEMP, W               ; restore W first
    movff   STATUS_TEMP, STATUS     ; restore STATUS last without affecting W
			endm

; Delay X microseconds
delayMS     macro	countReg, MS
    local   Again
    movlw   MS
    movwf	countReg
Again
    decfsz	countReg, F
    goto	Again
			endm

; Display Table Data on LCD
DispTable	macro	TableVar
		local	Again
		movlw	upper TableVar	; Move Table<20:16> into TBLPTRU
		movwf	TBLPTRU
		movlw	high TableVar	; Move Table<15:8> into TBLPTRH
		movwf	TBLPTRH
		movlw	low TableVar	; Move Table<7:0> into TBLPTRL
		movwf	TBLPTRL
		tblrd*					; Read byte at TBLPTR and copy to TABLAT
		movf	TABLAT, W		; Move byte into W
Again
		call	WR_DATA			; Write byte to LCD
		tblrd+*					; Increment pointer
		movf	TABLAT, W		; Move new byte into W
		bnz		Again			; Keep going until the end (0 byte)
				endm

; Display Operation Log on LCD
DispOpLog	macro	addrH, addrL
		local		Again, P, OneF, TwoF, ThreeF, N, Write, Skip, WriteSkip, Finish
		clrf		SkipCount
Again
		; Put a space every 3 Writes
		movlw		d'3'
		cpfslt		SkipCount
		goto		Skip
		; Read EEPROM data
		ReadEEPROM	EEPROM_REG, addrH, addrL
		btfsc		EEPROM_REG, 7	; If bit 7 is (1) Finish
		goto		Finish
		; Check if FL was present
		btfss		EEPROM_REG, 3	; If bit 3 is (1), there was a FL
		goto		N				; If bit 3 is (0), there was no FL, so print N
		; Count how many LED was on
		btfsc		EEPROM_REG, 2
		incf		LED_Count
		btfsc		EEPROM_REG, 1
		incf		LED_Count
		btfsc		EEPROM_REG, 0
		incf		LED_Count
		; Branch to appropriate #
		movlw		d'3'
		cpfslt		LED_Count
		goto		P
		movlw		d'2'
		cpfslt		LED_Count
		goto		OneF
		movlw		d'1'
		cpfslt		LED_Count
		goto		TwoF
		goto		ThreeF
P
		movlw		0x50
		goto		Write
OneF
		movlw		0x31
		goto		Write
TwoF
		movlw		0x32
		goto		Write
ThreeF
		movlw		0x33
		goto		Write
N
		movlw		0x4E			; ASCII 'N'
		goto		Write
Write
		call		WR_DATA			; Write Char to LCD
		incf		addrL			; Next byte in EEPROM
		incf		SkipCount
		clrf		LED_Count
		goto		Again
Skip
		movlw		0x20			; ASCII ' '
		goto		WriteSkip
WriteSkip
		call		WR_DATA
		clrf		SkipCount
		goto		Again
Finish
		nop
		endm

; Write word to EEPROM
WriteEEPROM macro   word, addrH, addrL
        movff       addrH, EEADRH   ; Set high address
        movff       addrL, EEADR    ; Set low address
        movff       word,  EEDATA   ; Set word data

		btfsc		EECON1, WR		; Check if WR = 0
		bra			$-2

        bcf         EECON1, EEPGD   ; Point to DATA memory                                              ; ADDED AN E
        bcf         EECON1, CFGS    ; Access EEPROM
        bsf         EECON1, WREN    ; Enable writes
        bcf         INTCON, GIE     ; Disable interrupts
		bcf			PIR2, EEIF																			; lolwut?

        movlw       0x55
        movwf       EECON2          ; Write 55h
        movlw       0xAA            ;
        movwf       EECON2          ; Write 0xAA
        bsf         EECON1, WR      ; Set WR bit to begin write
        btfsc		EECON1, WR
		bra			$-2

		bsf         INTCON, GIE     ; Enable interrupts
        bcf         EECON1, WREN    ; Diable writes on write complete (EEIF set)
            endm

; Read EEPROM into file
ReadEEPROM  macro   file, addrH, addrL
        movff       addrH, EEADRH   ; Set high address
        movff       addrL, EEADR    ; Set low address
        bcf         EECON1, EEPGD   ; Point to DATA memory
        bcf         EECON1, CFGS    ; Access EEPROM
        bsf         EECON1, RD      ; EEPROM Read
        movff       EEDATA, file    ; file <- EEDATA
            endm

; Change FSM State
ChangeState	macro	KeyCode, NextState
		local		Next, NotNext																		; Need this to bypass 'out of range' error
		movlw		KeyCode			; If 'KeyCode' was pressed
		subwf		KEY
		bz			Next			; Go to 'NextState'
		bra			NotNext
Next								; Before going to the next state,                                 ; FIX THIS STUFF LATER WITH CPFSEQ
		clrf		PORTA			; Clear all Pins
        clrf		PORTB
        clrf		PORTC
        clrf		PORTD
        clrf        InOperation     ; Not in operation anymore
		goto		NextState
NotNext
		movff		KEY_Temp, KEY	; Restore KEY for next check
			endm

StepMotor   macro
        call        Step1
        call        StepDelay
        call        Step2
        call        StepDelay
        call        Step3
        call        StepDelay
        call        Step4
        call        StepDelay
            endm

; ****************************************************************************
; VECTORS
; ****************************************************************************
	org		0x0000
	goto	Init

	org		0x08				;high priority ISR
	goto	ISR_HIGH

	org		0x18				;low priority ISR
	retfie

; ****************************************************************************
; INTERRUPT SERVICE ROUTINE
; ****************************************************************************

ISR_HIGH
    saveContext
    btfss   INTCON3, INT1IF        ; If KEYPAD interrupt, skip return
    goto    END_ISR_HIGH

    movlw   0xFF                ; If in operation, skip return
    cpfseq  InOperation
    goto    END_ISR_HIGH

    swapf   PORTB, W            ; Read PORTB<7:4> into W<3:0>
    andlw   0x0F
    movwf   KEY_ISR            ; Put W into KEY_ISR
    movlw   keyStar             ; Put keyStar into W to compare to KEY_ISR
    cpfseq  KEY_ISR            ; If key was '*', skip return
    goto    END_ISR_HIGH

    clrf    TOSU                   ; Reset program counter
    clrf    TOSH
    clrf    TOSL

END_ISR_HIGH
    bcf     INTCON3, INT1IF        ; Clear flag for next interrupt
    restContext
	retfie

; ****************************************************************************
; TABLES
; ****************************************************************************

MainMenu_L1		db	"Standby", 0
MainMenu_L2		db	"1:30PM, 2/3/2014", 0
Operation_L1	db	"In Operation...", 0
OpLog_L1		db	"Oprtn. Time: 89s", 0
OpLog_L2		db	"12:00PM, 2/3/14"
OpLogDetails_L1	db	"#: 123 456 789", 0
OpLogDetails_L2	db	"?: ", 0
PermLog_L1		db	"Permanent Logs", 0
PermLog_L2		db	"1  2  3  4", 0
PermLog1_L1		db	"Permanent Log 1", 0
PCInter_L1		db	"PC Interface", 0
PCInter_L2		db	"Connect to PC...", 0

; ****************************************************************************
; MAIN PROGRAM
; ****************************************************************************
        CODE
Init
        ;clrf		INTCON         ; No interrupts
        ;clrf		TRISA          ; All port A is output

        movlw       b'00110000'    ; Set PORTA4 as input
        movwf       TRISA

        movlw       b'00000111'    ; Set PORTE<0:2> as input
        movwf       TRISE

        movlw		b'11111111'    ; Set required keypad inputs (RB0 is interrupt)
        movwf		TRISB

		movlw		b'00011000'
        clrf		TRISC          ; All port C is output
        clrf		TRISD          ; All port D is output
        clrf		LATA
        clrf		LATB
        clrf		LATC
        clrf		LATD
		call		InitLCD

        bcf         RCON, IPEN          ; Legacy mode interrupts
        bsf         INTCON, GIE         ; Allow global interrupt
        bsf         INTCON3, INT1IE     ; enable INT1 int flag bit on RB1
        bsf         INTCON2, INTEDG1    ; set INTEDG1 to detect rising edge

		clrf		EEPROM_H			; Initialize EEPROM address
		clrf		EEPROM_L
		clrf		FL_count
		clrf		TrayEncoder
		clrf		LED_Count
		clrf		SkipCount


; STANDBY STATE
Standby
		call		ClrLCD
		DispTable	MainMenu_L1
		call LCD_L2
		DispTable	MainMenu_L2
Stay_Standby
		call		ReadKEY						; Wait for key inputs
		ChangeState keyA, Operation				; A for Operation
		ChangeState keyB, OpLog					; B for OpLog
		ChangeState keyC, PermLogMenu			; C for PermLog
		ChangeState	keyD, PCInter				; D for PC Interface
		bra			Stay_Standby

; OPERATION STATE
Operation
        setf        InOperation
		call		ClrLCD
		DispTable	Operation_L1
		call		LCD_L2
		DispTable	MainMenu_L2
        clrf        FL_count                    ; Clear FL_count to 0

        ;TESTING
;        bsf         ArmSol
;forever
;        StepMotor
;        goto        forever

FIND_FIRST_FL                                   ; Keep rotating until break beam is (1) not broken
        btfss       BreakBeam
        goto        FIND_FL
			; Step Motor 1 degree
        goto        FIND_FIRST_FL

FIND_FL
        btfss       BreakBeam                   ; Do nothing if beam is not broken (1)
        goto        FOUND_FL                    ; If beam is broken (0) go to FOUND_FL
        dcfsnz      TrayEncoder                 ; Do nothing is tray encoder is not(0)
        goto        NO_MORE_FL                  ; If trayEncoder counted 50 degrees (0) go to NO_MORE_FL
			; Step Motor 1 degree
        goto        FIND_FL

FOUND_FL
        clrf        TrayEncoder                 ; Reset tray encoder since FL was found
        movlw       1
        addwf       FL_count                    ; Increment FL_count
        ; TURN ON LED
        bsf			ArmSol						; Pull arm down
        bsf         GripSol                     ; Pull grip in
        call        TurnGripCW                  ; Turn grip CW
        ; TAKE DATA FROM PHOTO SENSORS
        call        PhotoData
        ; TURN OFF LED
        call        TurnGripCCW                 ; Turn grip CCW
        bcf         GripSol                     ; Release grip
        bcf         ArmSol                      ; Release arm
        ; CHECK EXIT CONDITIONS
        movlw       9                           ; Don't exit if under 9 count
        cpfslt      FL_count
        goto        EXIT_OP                     ; Exit if FL_count is 9
			; Step motor 20 degrees
        goto        FIND_FL                     ; Keep looking for FL if under 9 count

NO_MORE_FL
        movlw       1                           ; Increment FL_count
        addwf       FL_count
        call        NoPhotoData					; Take data for N/A FL
        movlw       9                           ; Don't exit if under 9 count
        cpfslt      FL_count
        goto        EXIT_OP                     ; Exit if FL_count is 9

EXIT_OP
			;Stop Timer
        goto        OpLog

Stay_Operation
		call		ReadKEY						; Wait for key inputs
		ChangeState	keyStar, Standby			; * for Back (Standby)
		bra			Stay_Operation

; OPERATION LOG STATE
OpLog
		call		ClrLCD
		DispTable	OpLog_L1
		call LCD_L2
		DispTable	OpLog_L2
Stay_OpLog
		call		ReadKEY
		ChangeState	key1, OpLogDetails
		ChangeState keyStar, Standby
		bra			Stay_OpLog
OpLogDetails
		call		ClrLCD
		DispTable	OpLogDetails_L1
		call LCD_L2
		DispTable	OpLogDetails_L2
		clrf		EEPROM_H					; Display recent operation log
		clrf		EEPROM_L
		DispOpLog	EEPROM_H, EEPROM_L
StayOpLogDetails
		call		ReadKEY
		ChangeState keyStar, OpLog
		bra			Stay_OpLog

; PERMANENT LOG STATE
PermLogMenu
		call		ClrLCD
		DispTable	PermLog_L1
		call LCD_L2
		DispTable	PermLog_L2
Stay_PermLogMenu
		call		ReadKEY
		ChangeState	key1, PermLog1
		ChangeState keyStar, Standby
		bra			Stay_PermLogMenu
PermLog1
		call		ClrLCD
		DispTable	PermLog1_L1
		call LCD_L2
		DispTable	OpLog_L2
Stay_PermLog1
		call		ReadKEY
		ChangeState	key1, PermLog1Details
		ChangeState keyStar, PermLogMenu
		bra			Stay_PermLog1
PermLog1Details
		call		ClrLCD
		DispTable	OpLogDetails_L1
		call LCD_L2
		DispTable	OpLogDetails_L2
Stay_PermLog1Details
		call		ReadKEY
		ChangeState keyStar, PermLog1
		bra			Stay_PermLog1Details

; PC INTERFACE STATE
PCInter
	call ClrLCD
	DispTable	PCInter_L1
	call LCD_L2
	DispTable	PCInter_L2
Stay_PCInter
	call		ReadKEY
	ChangeState	keyStar, Standby
	bra			Stay_PCInter


; ****************************************************************************
; SUBROUTINES
; ****************************************************************************



; DATA SUBROUTINES
PhotoData                                       ; CALLED FROM FOUND_FL
        movff       PORTE, PhotoInput           ; Take input from photo sensors
        bsf         PhotoInput, 3               ; Set 1 for break beam since FL was found
        movlw       b'00001111'
        andwf       PhotoInput
        WriteEEPROM PhotoInput, EEPROM_H, EEPROM_L
        incf		EEPROM_L
        return

NoPhotoData
        clrf        PhotoInput
        WriteEEPROM PhotoInput, EEPROM_H, EEPROM_L
        incf		EEPROM_L
        return

; STEPPER MOTOR SUBROUTINES
Step1
        bsf         Step1a
        bcf         Step1b
        bcf         Step2a
        bcf         Step2b
        return
Step2
        bcf         Step1a
        bcf         Step1b
        bsf         Step2a
        bcf         Step2b
        return
Step3
        bcf         Step1a
        bsf         Step1b
        bcf         Step2a
        bcf         Step2b
        return
Step4
        bcf         Step1a
        bcf         Step1b
        bcf         Step2a
        bsf         Step2b
        return
StepDelay
		movlw	StepDelayVal
		movwf	stepDelay1,0
		movlw	StepDelayVal
		movwf	stepDelay2,0
StepDelayLoop
		decfsz	stepDelay1, f           ; Delay1 counts down first
		goto	d2
		decfsz	stepDelay2, f           ; Delay2 counts down when Delay1 = 0
d2		goto	StepDelayLoop
		return

; Turn Gripper Subroutines
TurnGripCW
        return

TurnGripCCW
        return

; Read Keypad Input
ReadKEY
WaitKey
		btfss		PORTB,1     ;Wait until data is available from the keypad
        goto		WaitKey		;Once a key is pressed,
        swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F		;Mask 00001111
		movwf		KEY			;Save result in KEY
		movwf		KEY_Temp
		btfsc		PORTB,1     ;Wait until key is released
        goto		$-2			;Back 1 instruction
		return
	END