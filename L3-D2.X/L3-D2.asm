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
#define     BreakBeam           PORTA, 4
#define		ArmSol              LATC, 5
#define     GripSol             LATC, 2
#define     Step1a              LATA, 0
#define     Step1b              LATA, 1
#define     Step2a              LATA, 2
#define     Step2b              LATA, 3
#define     StepDelayVal        0x1F

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

delayReg	equ		0x30

KEY			equ		0x50
KEY_Temp	equ		0x51
KEY_ISR     equ     0x52

InOperation equ     0x60
TrayEncoder equ     0x61
stepDelay1  equ     0x62
stepDelay2  equ     0x63


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

; Write word to EEPROM
WriteEEPROM macro   word, addrH, addrL
        movlw       addrH           ; Set high address
        movwf       EEADRH
        movlw       addrL           ; Set low address
        movwf       EEADR
        movlw       word            ; Set word data
        movwf       EEDATA
        bcf         EECON1, EPGD    ; Point to DATA memory
        bcf         EECON1, CFGS    ; Access EEPROM
        bsf         EECON1, WREN    ; Enable writes

        bcf         INTCON, GIE     ; Disable interrupts
        movlw       55h
        movwf       EECON2          ; Write 55h
        movlw       0xAA            ;
        movwf       EECON2          ; Write 0xAA
        bsf         EECON1, WR      ; Set WR bit to begin write
        bsf         INTCON, GIE     ; Enable interrupts

        bcf         EECON1, WREN    ; Diable writes on write complete (EEIF set)
            endm

; Read EEPROM into WREG
ReadEEPROM  macro   addrH, addrL
        movlw       addrH           ; Set high address
        movwf       EEADRH
        movlw       addrL           ; Set low address
        movwf       EEADR
        bcf         EECON1, EEPGD   ; Point to DATA memory
        bcf         EECON1, CFGS    ; Access EEPROM
        bsf         EECON1, RD      ; EEPROM Read
        movf        EEDATA, WREG    ; W <- EEDATA
            endm

; Change FSM State
ChangeState	macro	KeyCode, NextState
		local		Next, NotNext	; Need this to bypass 'out of range' error
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
    bsf     ArmSol                                                                                       ; TESTING
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
OpLogDetails_L2	db	"?: PP1 P23 PNN", 0
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
        clrf		INTCON         ; No interrupts
        clrf		TRISA          ; All port A is output
        movlw       b'00010000'    ; Set RA4 as input
        movwf       TRISA
        movlw		b'11111111'    ; Set required keypad inputs (RB0 is interrupt)
        movwf		TRISB
        clrf		TRISC          ; All port C is output
        clrf		TRISD          ; All port D is output
        clrf		LATA
        clrf		LATB
        clrf		LATC
        clrf		LATD
		call		InitLCD

        bcf         RCON, IPEN          ; Legacy mode interrupts
        bsf         INTCON, GIE         ; Allow global interrupt
        bsf         INTCON3, INT1IE     ; enable INT0 int flag bit on RB0
        bsf         INTCON2, INTEDG1    ; set INTEDG0 to detect rising edge


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
		call LCD_L2
		DispTable	MainMenu_L2

        ;TESTING
        ;bsf         ArmSol
forever
        StepMotor
        goto        forever
;
;CHECK_BEAM
;        btfss       BreakBeam                   ; Do nothing if beam is set (1)
;        goto        FOUND_FL                    ; If beam is broken (0)
;        dcfsnz      TrayEncoder                 ; Do nothing is tray encoder is not(0)
;        goto        NO_MORE_FL                  ; If trayEncoder counted 50 degrees (0)
;        goto        FIND_FL                     ; Else find the next flashlight
;FIND_FL
;        StepMotor
;        goto        CHECK_BEAM
;FOUND_FL
;        clrf        TrayEncoder                 ; Reset tray encoder since FL was found
;        ; TURN ON LED
;        bsf			ArmSol						; Pull arm down
;        bsf         GripSol                     ; Pull grip in
;        call        TurnGripCW                  ; Turn grip CW
;                    ; take some data somehow lol
;        ; TURN OFF LED
;        call        TurnGripCCW                 ; Turn grip CCW
;        bcf         GripSol                     ; Release grip
;        bcf         ArmSol                      ; Release arm
;NO_MORE_FL
;
;
;Stay_Operation
;		call		ReadKEY						; Wait for key inputs
;		ChangeState	keyStar, Standby			; * for Back (Standby)
;		bra			Stay_Operation

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