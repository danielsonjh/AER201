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
#define		RS      LATD, 2        ; for v 1.0 used PORTD.3
#define		E       LATD, 3        ; for v 1.0 used PORTD.2
#define		ArmSol	LATC, 5

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

temp_lcd	EQU     0x20           ; buffer for Instruction
dat			EQU     0x21           ; buffer for data
delay1		EQU		0x25
delay2		EQU		0x26
delay3		EQU		0x27

delayReg	equ		0x30

KEY			equ		0x50
KEY_Temp	equ		0x51

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

; Change LCD Lines
LCD_L1		macro
		movlw		B'10000000'
		call		WR_INS
			endm
LCD_L2		macro
		movlw		B'11000000'
		call		WR_INS
			endm

; Change FSM State
ChangeState	macro	KeyCode, NextState
		local		Next, NotNext	; Need this to bypass 'out of range' error
		movlw		KeyCode			; If 'KeyCode' was pressed
		subwf		KEY
		bz			Next			; Go to 'NextState'
		bra			NotNext
Next								; Before going to the next state,
		clrf		PORTA			; Clear all Pins
        clrf		PORTB
        clrf		PORTC
        clrf		PORTD
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
	retfie

; ****************************************************************************
; INTERRUPT SERVICE ROUTINE
; ****************************************************************************

ISR_HIGH
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
OpLogDetails_L2	db	"?: PP1 P23 PNN", 0
PermLog_L1		db	"Permanent Logs", 0
PermLog_L2		db	"1  2  3  4", 0
PermLog1_L1		db	"Permanent Log 1", 0
PCInter_L1		db	"PC Interface", 0
PCInter_L2		db	"Connect to PC...", 0




; ****************************************************************************
; MAIN PROGRAM
; ****************************************************************************

        code
Init
        clrf		INTCON         ; No interrupts
        clrf		TRISA          ; All port A is output
        movlw		b'11110010'    ; Set required keypad inputs
        movwf		TRISB
        clrf		TRISC          ; All port C is output
        clrf		TRISD          ; All port D is output
        clrf		LATA
        clrf		LATB
        clrf		LATC
        clrf		LATD
		call		InitLCD

; STANDBY STATE
Standby
		call		ClearLCD
		DispTable	MainMenu_L1
		LCD_L2
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
		call		ClearLCD
		DispTable	Operation_L1
		LCD_L2
		DispTable	MainMenu_L2
		bsf			ArmSol						; Turn on Arm Solenoid
Stay_Operation
		call		ReadKEY						; Wait for key inputs
		ChangeState	keyStar, Standby			; * for Back (Standby)
		bra			Stay_Operation

; OPERATION LOG STATE
OpLog
		call		ClearLCD
		DispTable	OpLog_L1
		LCD_L2
		DispTable	OpLog_L2
Stay_OpLog
		call		ReadKEY
		ChangeState	key1, OpLogDetails
		ChangeState keyStar, Standby
		bra			Stay_OpLog
OpLogDetails
		call		ClearLCD
		DispTable	OpLogDetails_L1
		LCD_L2
		DispTable	OpLogDetails_L2
StayOpLogDetails
		call		ReadKEY
		ChangeState keyStar, OpLog
		bra			Stay_OpLog

; PERMANENT LOG STATE
PermLogMenu
		call		ClearLCD
		DispTable	PermLog_L1
		LCD_L2
		DispTable	PermLog_L2
Stay_PermLogMenu
		call		ReadKEY
		ChangeState	key1, PermLog1
		ChangeState keyStar, Standby
		bra			Stay_PermLogMenu
PermLog1
		call		ClearLCD
		DispTable	PermLog1_L1
		LCD_L2
		DispTable	OpLog_L2
Stay_PermLog1
		call		ReadKEY
		ChangeState	key1, PermLog1Details
		ChangeState keyStar, PermLogMenu
		bra			Stay_PermLog1
PermLog1Details
		call		ClearLCD
		DispTable	OpLogDetails_L1
		LCD_L2
		DispTable	OpLogDetails_L2
Stay_PermLog1Details
		call		ReadKEY
		ChangeState keyStar, PermLog1
		bra			Stay_PermLog1Details

; PC INTERFACE STATE
PCInter
	call ClearLCD
	DispTable	PCInter_L1
	LCD_L2
	DispTable	PCInter_L2
Stay_PCInter
	call		ReadKEY
	ChangeState	keyStar, Standby
	bra			Stay_PCInter


; ****************************************************************************
; SUBROUTINES
; ****************************************************************************

;****************************************
;		Write command to LCD
;		Input  : W
;		output : -
;****************************************
WR_INS
		bcf		RS	  				; clear Register Status bit
		movwf	temp_lcd			; store instruction
		andlw	0xF0			  	; mask 4 bits MSB
		movwf	LATD			  	; send 4 bits MSB

		bsf		E					; pulse enable high
		swapf	temp_lcd, WREG		; swap nibbles
		andlw	0xF0			  	; mask 4 bits LSB
		bcf		E
		movwf	LATD			  	; send 4 bits LSB
		bsf		E					; pulse enable high
		bcf		E
		call	delay5ms

		return

;***************************************
;		Write data to LCD
;		Input  : W
;		Output : -
;***************************************
WR_DATA
		bcf		RS					; clear Register Status bit
        movwf   dat					; store character
        movf	dat, WREG
		andlw   0xF0			  	; mask 4 bits MSB
        addlw   4					; set Register Status
        movwf   PORTD			  	; send 4 bits MSB

		bsf		E					; pulse enable high
        swapf   dat, WREG		  	; swap nibbles
        andlw   0xF0			  	; mask 4 bits LSB
		bcf		E
        addlw   4					; set Register Status
        movwf   PORTD			  	; send 4 bits LSB
		bsf		E					; pulse enable high
		bcf		E

		call	delay44us

        return

ClearLCD
		movlw	B'00000001'
		call	WR_INS
		return

SwitchLineLCD
		movlw	B'11000000'
		call	WR_INS
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

;******************************************************************************
; Delay44us (): wait exactly  110 cycles (44 us)
; <www.piclist.org>

delay44us
		movlw	0x23
		movwf	delay1, 0

Delay44usLoop

		decfsz	delay1, f
		goto	Delay44usLoop
		return

delay5ms
		movlw	0xC2
		movwf	delay1,0
		movlw	0x0A
		movwf	delay2,0

Delay5msLoop
		decfsz	delay1, f
		goto	d2
		decfsz	delay2, f
d2		goto	Delay5msLoop
		return
	end