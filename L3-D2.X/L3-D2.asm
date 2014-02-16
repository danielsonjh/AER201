#include <p18f4620.inc>
#include <lcd18.inc>
#include <rtc_macros.inc>
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
#define		GripMotorCW			LATC, 1
#define		GripMotorCCW		LATC, 0
#define     Step1a              LATA, 0
#define     Step1b              LATA, 1
#define     Step2a              LATA, 2
#define     Step2b              LATA, 3																		 ; EDIT FOR STEP DELAY
#define     StepDelayVal        0x1F
#define		OpDelay				d'20'
#define		GripMotorDelay		0x04
#define		StepSize			0x01
#define		TimerConstantH		0x9E			; 2^16 - 25000
#define		TimerConstantL		0x58

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
Counter		equ		0x02
Temp		equ		0x03
TempBCD		equ		0x04

; 0x20 to 0x27 for LCD

delayReg	equ		0x30
reg100us	equ		0x31
reg50ms		equ		0x32

KEY			equ		0x50
KEY_Temp	equ		0x51
KEY_ISR     equ     0x52

InOperation equ     0x60
TrayEncoder equ     0x61
stepDelay1  equ     0x62
stepDelay2  equ     0x63
FL_count    equ     0x64
PhotoInput  equ     0x65

; 0x71 to 0x78 for RTC

EEPROM_CLEAR	equ	0x79
EEPROM_H    equ     0x80
EEPROM_L    equ     0x81
EEPROM_REG	equ		0x82
LED_Count	equ		0x83
SkipCount	equ		0x84

Op_Seconds	equ		0x90
Op_Interrupts	equ	0x91


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

; Increment file as 2 digit BCD
incf_BCD	macro		file
	local	justones, hundred, end_incf_BCD
	movff	file, TempBCD
	movff	file, Temp
	; Process 1's digit BCD
	movlw	0x0F
	andwf	Temp					; Mask lower nibble of Temp
	movlw	d'9'
	; If 1's is 9:
	cpfseq	Temp					; Skip if 1's digit is 9
	goto	justones
	movlw	b'00010000'				; Increment 10's digit of TempBCD
	addwf	TempBCD
	movlw	0xF0					; Clear 1's digit of TempBCD
	andwf	TempBCD
	goto	hundred
	; If 1's is less than 9:
justones
	incf	TempBCD
	goto	end_incf_BCD
	; If A0 (100) happens:
hundred
	movlw	0xA0
	cpfslt	TempBCD					; If TempBCD's 10's digit is less than 100, skip
	clrf	TempBCD					; If TempBCD's 10's digit is 100, clear everything
end_incf_BCD
	movff	TempBCD, file
			endm

; Delay 50xN milliseconds
Delay50xNms macro	countReg, N
    local   Again
    movlw   N
    movwf	countReg
Again
	call	Delay50ms
    decfsz	countReg
    goto	Again
			endm

; Write RTC Data
WriteRTC	macro
		movff		0x77, W
		call		WR_DATA
		movff		0x78, W
		call		WR_DATA
			endm

; Step Tray X Degrees
XDegreeStep	macro	X
		local	Again
		movlw	X / StepSize
		movwf	Counter
Again
		call	StepMotor
		decfsz	Counter
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
		; Check if there is No Data first
		ReadEEPROM	EEPROM_REG, addrH, addrL
		movlw		0xFF
		cpfseq		EEPROM_REG
		goto		Again
		DispTable	NoData
		goto		Finish
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

; ****************************************************************************
; VECTORS
; ****************************************************************************
	org		0x0000
	goto	Init

	org		0x08				;high priority ISR
	goto	ISR_HIGH

	org		0x18				;low priority ISR
	goto	ISR_LOW
	retfie

; ****************************************************************************
; INTERRUPT SERVICE ROUTINE
; ****************************************************************************

ISR_HIGH
;	saveContext								; 3
;	; Reset Timer
;	movlw		TimerConstantH				; 1
;	addwf		TMR0H						; 1
;	movlw		TimerConstantL				; 1
;	addwf		TMR0L						; 1
;	movlw		d'9'						; 1
;	subwf		TMR0L						; 1
;	; TIMER INTERRUPT
;	btfss		INTCON, TMR0IF
;	goto		END_ISR_HIGH
;	; Increment as BCD interrupts and seconds
;	incf_BCD	Op_Interrupts
;	movlw		d'0'
;	cpfsgt		Op_Interrupts
;	incf_BCD	Op_Seconds
;END_ISR_HIGH
;	bcf			INTCON, TMR0IF
;	restContext
	retfie

ISR_LOW
    saveContext
	; KEYPAD INTERRUPT
    btfss		INTCON3, INT1IF			; If KEYPAD interrupt, skip return
    goto		END_ISR_LOW
	; Check operation status
    movlw		0xFF					; If in operation, skip return
    cpfseq		InOperation
    goto		END_ISR_LOW
	; Process KEY
    swapf		PORTB, W				; Read PORTB<7:4> into W<3:0>
    andlw		0x0F
    movwf		KEY_ISR					; Put W into KEY_ISR
    movlw		keyStar					; Put keyStar into W to compare to KEY_ISR
    cpfseq		KEY_ISR					; If key was '*', skip return
    goto		END_ISR_LOW
	; Reset program counter
    clrf		TOSU
    clrf		TOSH
    clrf		TOSL
END_ISR_LOW
    bcf			INTCON3, INT1IF         ; Clear flag for next interrupt
    restContext
	retfie

; ****************************************************************************
; TABLES
; ****************************************************************************

MainMenu_L1		db	"Standby", 0
Operation_L1	db	"In Operation...", 0
Operation_L2	db	"*: Stop", 0
OpLog_L1		db	"Oprtn. Time: ", 0
OpLog_L2		db	"12:00PM, 2/3/14", 0
OpLogDetails_L1	db	"#: 123 456 789", 0
OpLogDetails_L2	db	"?: ", 0
PermLog_L1		db	"Permanent Logs", 0
PermLog_L2		db	"1  2  3  4", 0
PermLog1_L1		db	"Permanent Log 1", 0
PCInter_L1		db	"PC Interface", 0
PCInter_L2		db	"Connect to PC...", 0
NoData			db	"No Data", 0

; ****************************************************************************
; MAIN PROGRAM
; ****************************************************************************
        CODE
Init
		; Clear first 15 bytes of EEPROM (MIGHT NOT NEED THIS...)										; SLIGHT BUG WITH THIS WHEN RESETTING DURING A RUN AFTER 1 SUCCESSFUL RUN (Save PCL AFTER THIS)
		clrf		EEPROM_H
		clrf		EEPROM_L
		setf		EEPROM_CLEAR
		clrf		Counter
ClearNext
		WriteEEPROM EEPROM_CLEAR, EEPROM_H, EEPROM_L
		incf		Counter
		incf		EEPROM_L
		movlw		d'15'
		cpfseq		Counter
		goto		ClearNext
		; Setup I/O
        movlw       b'00110000'    ; Set PORTA<4:5> as input
        movwf       TRISA
        movlw       b'00000111'    ; Set PORTE<0:2> as input
        movwf       TRISE
        movlw		b'11111111'		; Set required keypad inputs (RB0 is interrupt)
        movwf		TRISB
		movlw		b'00011000'
		movwf		TRISC			; RC3, RC4 input for RTC
		; Clear Ports
        clrf		TRISD          ; All port D is output
        clrf		LATA
        clrf		LATB
        clrf		LATC
        clrf		LATD
		clrf		LATE
		; Setup Peripherals
		call		InitLCD
		call		i2c_common_setup
		;movlw		b'00001000'			; No 16-bit, internal, no prescaler
		;movwf		T0CON
		; Setup Interrupts
        bsf         RCON, IPEN          ; Priority mode interrupts
        bsf         INTCON, GIEH        ; Allow global interrupt
		bsf			INTCON, GIEL
        bsf         INTCON3, INT1IE     ; enable INT1 int flag bit on RB1
        bsf         INTCON2, INTEDG1    ; set INTEDG1 to detect rising edge
		;bsf			INTCON, TMR0IE
		; Clear FSR
		clrf		EEPROM_H			; Initialize EEPROM address
		clrf		EEPROM_L
		clrf		Counter
		clrf		FL_count
		clrf		TrayEncoder
		clrf		LED_Count
		clrf		SkipCount
		clrf		InOperation

		; TEST OPLOG
		bsf			LATE, 0
		bsf			LATE, 1
		bsf			LATE, 2

		; SET UP RTC
		rtc_resetAll
		; 11:48PM, 12/31/2094
		rtc_set 0x00, b'00000000'		; Set seconds to 0
		rtc_set 0x01, b'01001000'		; Set minutes (48)
		rtc_set	0x02, b'01110001'		; Set hours (11PM)
		rtc_set 0x04, b'00110001'		; Set day (31)
		rtc_set	0x05, b'00010010'		; Set month (12)
		rtc_set 0x06, b'10010100'		; Set year (94)


; STANDBY STATE
Standby
		call		ClrLCD
		DispTable	MainMenu_L1
Stay_Standby
		call		Read_KEY_RTC				; Wait for key inputs
		ChangeState keyA, Operation				; A for Operation
		ChangeState keyB, OpLog					; B for OpLog
		ChangeState keyC, PermLogMenu			; C for PermLog
		ChangeState	keyD, PCInter				; D for PC Interface
		bra			Stay_Standby

; OPERATION STATE
Operation
		; Start Timer
		clrf		TMR0H						; 1
		clrf		TMR0L						; 1
		movlw		TimerConstantH				; 1
		addwf		TMR0H						; 1
		movlw		TimerConstantL				; 1
		addwf		TMR0L						; 1
		bsf			T0CON, TMR0ON				; Turn on timer
		; Display
        setf        InOperation
		call		ClrLCD
		DispTable	Operation_L1
		call		LCD_L2
		DispTable	Operation_L2

        ;TESTING
;        bsf         ArmSol
;forever
;        StepMotor
;        goto        forever

FIND_FIRST_FL                                   ; Keep rotating until break beam is (1) not broken
        btfss       BreakBeam
        goto        FIND_FL
		XDegreeStep 1
        goto        FIND_FIRST_FL

FIND_FL
        btfss       BreakBeam                   ; Do nothing if beam is not broken (1)
        goto        FOUND_FL                    ; If beam is broken (0) go to FOUND_FL
        dcfsnz      TrayEncoder                 ; Do nothing is tray encoder is not(0)
        goto        NO_MORE_FL                  ; If trayEncoder counted 50 degrees (0) go to NO_MORE_FL
		XDegreeStep 1
        goto        FIND_FL

FOUND_FL
        clrf        TrayEncoder                 ; Reset tray encoder since FL was found
        incf		FL_count
        ; TURN ON LED
        bsf			ArmSol						; Pull arm down
		Delay50xNms	delayReg, OpDelay			; Delay
        bsf         GripSol                     ; Pull grip in
		Delay50xNms	delayReg, OpDelay			; Delay
        call        TurnGripCW                  ; Turn grip CW
        ; TAKE DATA FROM PHOTO SENSORS
        call        PhotoData
		Delay50xNms	delayReg, OpDelay			; Delay 350ms
        ; TURN OFF LED
        call        TurnGripCCW                 ; Turn grip CCW
		Delay50xNms	delayReg, OpDelay			; Delay
        bcf         GripSol                     ; Release grip
		Delay50xNms	delayReg, OpDelay			; Delay
        bcf         ArmSol                      ; Release arm
		Delay50xNms	delayReg, OpDelay			;	---->											; TAKE THIS OUT LATER, FOR TESTING RIGHT NOW
        ; CHECK EXIT CONDITIONS
        movlw       9                           ; Don't exit if under 9 count
        cpfslt      FL_count					; Skip if less than 9 FL counted
        goto        EXIT_OP                     ; Exit if FL_count is 9
		XDegreeStep	20
        goto        FIND_FL                     ; Keep looking for FL if under 9 count

NO_MORE_FL
        incf		FL_count
        call        NoPhotoData					; Take data for N/A FL
        movlw       9                           ; Don't exit if under 9 count
        cpfslt      FL_count
        goto        EXIT_OP                     ; Exit if FL_count is 9

EXIT_OP
		;Stop Timer
		bcf			T0CON, TMR0ON
        goto        OpLog

Stay_Operation
		call		ReadKEY						; Wait for key inputs
		ChangeState	keyStar, Standby			; * for Back (Standby)
		bra			Stay_Operation

; OPERATION LOG STATE
OpLog
		call		ClrLCD
		DispTable	OpLog_L1
		; Display OpTime
		; 10's seconds
		swapf		Op_Seconds, Temp			; Swap and mask upper nibble of Op_Seconds
		movlw		0x0F
		andwf		Temp						; Temp = upper nibble of Op_Seconds
		movff		Temp, W						; W = Temp
		addlw		0x30						; Convert to ASCII
		call		WR_DATA
		; 1's seconds
		movff		Op_Seconds, Temp			; Mask lower nibble of Op_Seconds
		movlw		0x0F
		andwf		Temp						; Temp = lower nibble of Op_Seconds
		movff		Temp, W						; W = Temp
		addlw		0x30						; Convert to ASCII
		call		WR_DATA
		; Write '.'
		movlw		0x2E
		call		WR_DATA
		; .1's seconds
		swapf		Op_Interrupts, Temp			; Swap and mask upper nibble of Op_Interrupts
		movlw		0x0F
		andwf		Temp						; Temp = upper nibble of Op_Interrupts
		movff		Temp, W						; W = Temp
		addlw		0x30						; Convert to ASCII
		call		WR_DATA
		; Write 's'
		movlw		0x73
		call		LCD_L2
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

; DELAYS
Delay100us							; delay 497 cycles (10MHz Clock)
		movlw		0xA4			; 1 cycle
		movwf		reg100us		; 1 cycle
loop100us
		decfsz		reg100us		; (3 * 164) - 1 = 491 cycles
		goto		loop100us
		nop							; 1 cycle
		nop							; 1 cycle
		return						; 2 cycles

Delay50ms							; delay 125,000 cycles (10MHz Clock)
		movlw		0xF9			; 1 cycle
		movwf		reg50ms			; 1 cycle
loop50ms
		 call		Delay100us
		 decfsz		reg50ms			; (497 + 3) * 249 - 1 = 124,499 Cycles
		 goto		loop50ms
		 call		Delay100us		; 497 cycles
		 return						; 2 cycles


; DATA SUBROUTINES
PhotoData                                       ; CALLED FROM FOUND_FL
        movff       LATE, PhotoInput           ; Take input from photo sensors
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
StepMotor
        call        Step1
        call        StepDelay
        call        Step2
        call        StepDelay
        call        Step3
        call        StepDelay
        call        Step4
        call        StepDelay
		return

; Turn Gripper Subroutines
TurnGripCW
		bsf			GripMotorCW
		Delay50xNms	delayReg, GripMotorDelay
		bcf			GripMotorCW
        return

TurnGripCCW
		bsf			GripMotorCCW
		Delay50xNms	delayReg, GripMotorDelay
		bcf			GripMotorCCW
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

Read_KEY_RTC
WaitKeyRTC
		; Reset to second line
		call		LCD_L2
		; Display hours
		rtc_read	0x02
		WriteRTC
		movlw		0x3A		;ASCII ':'
		call		WR_DATA
		; Dispay minutes
		rtc_read	0x01
		WriteRTC
		movlw		0x20		;ASCII ' '
		call		WR_DATA
		; Display month
		rtc_read	0x05
		WriteRTC
		movlw		0x2F		; ASCII '/'
		call		WR_DATA
		; Display day
		rtc_read	0x04
		WriteRTC
		movlw		0x2F		; ASCII '/'
		call		WR_DATA
		; Display year
		movlw		0x32		; ASCII '2'
		call		WR_DATA
		movlw		0x30		; ASCII '0'
		call		WR_DATA
		rtc_read	0x06
		WriteRTC
		;Process KEY
		btfss		PORTB,1     ;Wait until data is available from the keypad
        goto		WaitKeyRTC	;Once a key is pressed,
        swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F		;Mask 00001111
		movwf		KEY			;Save result in KEY
		movwf		KEY_Temp
		btfsc		PORTB,1     ;Wait until key is released
        goto		$-2			;Back 1 instruction
		return

	END