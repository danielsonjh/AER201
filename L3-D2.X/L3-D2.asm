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
		CONFIG MCLRE = ON, LPT1OSC = OFF, PBADEN = OFF
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

#define		ArmSol              1
#define     GripSol             0

#define     BreakBeam           PORTC, 5

#define     Photo3              PORTC, 2
#define     Photo2              PORTC, 1
#define     Photo1              PORTC, 0

#define     Step1a              LATA, 0
#define     Step1b              LATA, 1
#define     Step2a              LATA, 2
#define     Step2b              LATA, 3
#define		GripMotorCCW        LATA, 4
#define		GripMotorCW			LATA, 5

#define		TimerConstantH		0x9E			; 2^16 - 25000
#define		TimerConstantL		0x58
; Permanent Log Controls
#define		TempEEPROM_L_Const	d'220'
#define		LastPermLog			d'168'
; Stepper Controls
#define     StepDelayVal        0x1F
; Operation Controls
#define		OpDelay				d'10'
#define     TrayStep            d'42'           ; 19.9 degrees = 24 * 0.83077
#define     MaxTrayStep         d'70'           ; 49.8 degrees
#define		GripMotorDelay		d'220'          ; GripMotorDelay X (ON + OFF)/5 ms
#define     GripMotorOn         d'7'
#define     GripMotorOff        d'3'
#define     SolDelay            d'120'
#define     ArmSolDelay         d'30'          ; Max Duty Cycle = (N - 1) / N
#define     GripSolDelay        d'50'          ; Min Duty Cycle = 1 / N

key1				equ		d'0'
key2				equ		d'1'
key3				equ		d'2'
keyA				equ		d'3'
key4				equ		d'4'
key5				equ		d'5'
key6				equ		d'6'
keyB				equ		d'7'
key7				equ		d'8'
key8				equ		d'9'
key9				equ		d'10'
keyC				equ		d'11'
keyStar				equ		d'12'
key0				equ		d'13'
keyHash				equ		d'14'
keyD				equ		d'15'

STATUS_TEMP			equ     0x00
W_TEMP				equ     0x01
Counter				equ		0x02
Counter2			equ		0x03
Temp				equ		0x04

; 0x20 to 0x27 for LCD

delayReg			equ		0x30
reg200us			equ		0x31
reg5us              equ     0x32
reg50ms				equ		0x33

KEY					equ		0x50
TempKEY				equ		0x51
KEY_ISR				equ     0x52

InOperation			equ     0x60
TrayEncoder			equ     0x61
StepCounter         equ     0x66
stepDelay1			equ     0x62
stepDelay2			equ     0x63
FL_count			equ     0x64
PhotoInput			equ     0x65

EEPROM_CLEAR		equ		0x79
EEPROM_H			equ		0x80
EEPROM_L			equ		0x81
EEPROM_REG			equ		0x82
TempEEPROM_H		equ		0x83
TempEEPROM_L		equ		0x84
LED_Count			equ		0x85
SkipCount			equ		0x86			; To count up to 3 for displaying ' '
OpLogFinishCount	equ		0x87			; To stop after 9 EEPROM Reads

Op_Seconds			equ		0x90
Op_Interrupts		equ		0x91

InTransfer			equ		0xA0
PCInter_PCL			equ		0xA1
PCInter_PCLATH		equ		0xA2
PCInter_PCLATU		equ		0xA3
TIMCNT				equ		0xA4
LPCNT				equ		0xA5
TDATA				equ		0xA6

; 0xB0 and onwards for RTC

    extern tens_digit, ones_digit

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
incf_BCD	macro		BCD
	local	justones, end_incf_BCD
	movff	BCD, Temp
	; Process 1's digit BCD
	movlw	0x0F
	andwf	Temp					; Temp = lower nibble of BCD
	movlw	d'9'
	; If 1's is 9:
	cpfseq	Temp					; Skip if 1's digit is 9
	goto	justones				; Just increment lower nibble if 1's is less than 9
	movlw	b'00010000'				; Increment 10's digit of BCD
	addwf	BCD
	movlw	0xF0					; Clear 1's digit of BCD
	andwf	BCD
	; Check 100:
	movlw	0xA0
	cpfslt	BCD						; If BCD's 10's digit is less than 100, skip
	clrf	BCD						; If BCD's 10's digit is 100, clear everything
	goto	end_incf_BCD
	; If 1's is less than 9:
justones
	incf	BCD
	goto	end_incf_BCD
end_incf_BCD
	nop
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
		movff		tens_digit, WREG
		call		WR_DATA
		movff		ones_digit, WREG
		call		WR_DATA
			endm

; Step Tray X Degrees
XDegreeStep	macro	X
		local	Again
		movlw	X
		movwf	Counter
Again
		call	StepMotor
		incf	TrayEncoder
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

SendTable  macro   TableVar
		local	Again
    ; Write Transfer Time/Date
		movlw	upper TableVar          ; Move Table<20:16> into TBLPTRU
		movwf	TBLPTRU
		movlw	high TableVar           ; Move Table<15:8> into TBLPTRH
		movwf	TBLPTRH
		movlw	low TableVar            ; Move Table<7:0> into TBLPTRL
		movwf	TBLPTRL
		tblrd*                              ; Read byte at TBLPTR and copy to TABLAT
		movf	TABLAT, W                   ; Move byte into W
Again
		call	TransmitWaitUSART			; Write byte to LCD
		tblrd+*                             ; Increment pointer
		movf	TABLAT, W                   ; Move new byte into W
		bnz		Again                       ; Keep going until the end (0 byte)
            endm

; Display Operation Log on LCD
DispOpLog	macro	addrH, addrL
		local		Again, P, OneF, TwoF, ThreeF, N, Write, Skip, WriteSkip, Finish
		clrf		SkipCount
		clrf		OpLogFinishCount
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
		cpfslt		OpLogFinishCount
		goto		Finish
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
		incf		OpLogFinishCount
		goto		WriteSkip
WriteSkip
		call		WR_DATA
		clrf		SkipCount
		goto		Again
Finish
		nop
		endm


SendOpLog	macro	addrH, addrL
		local		Again, P, OneF, TwoF, ThreeF, N, Write, Skip, WriteSkip, Finish
		clrf		SkipCount
		clrf		OpLogFinishCount
		; Check if there is No Data first
		ReadEEPROM	EEPROM_REG, addrH, addrL
		movlw		0xFF
		cpfseq		EEPROM_REG
		goto		Again
		SendTable	NoData
		goto		Finish
Again
		; Put a space every 3 Writes
		movlw		d'3'
		cpfslt		SkipCount
		goto		Skip
		cpfslt		OpLogFinishCount
		goto		Finish
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
		call		TransmitWaitUSART			; Write Char to LCD
		incf		addrL			; Next byte in EEPROM
		incf		SkipCount
		clrf		LED_Count
		goto		Again
Skip
		movlw		0x20			; ASCII ' '
		incf		OpLogFinishCount
		goto		WriteSkip
WriteSkip
		call		TransmitWaitUSART
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

        bcf         EECON1, EEPGD   ; Point to DATA memory
        bcf         EECON1, CFGS    ; Access EEPROM
        bsf         EECON1, WREN    ; Enable writes
        bcf         INTCON, GIE     ; Disable interrupts
		bcf			PIR2, EEIF

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
		cpfseq		KEY
		goto		NotNext
Next								; Before going to the next state,
		clrf		LATA			; Clear all Pins
        clrf		LATB
        clrf		LATC
        clrf		LATD
		goto		NextState
NotNext
		nop
			endm

ReleaseSol  macro   Sol, Delay
        local       ReleaseSol_START, ReleaseSol_ON, ReleaseSol_OFF, END_ReleaseSol
        ; Use gradual PWM to release ArmSol slowly
        movlw       Delay
        movwf       delayReg           ; Say 10 ^ 2 ms to turn it off
        clrf        Counter2
ReleaseSol_START
        incf        Counter2
        dcfsnz      delayReg
        goto        END_ReleaseSol
        movlf       Delay, Counter            ; Start with 90% on
        movf        Counter2, W
        subwf       Counter                         ; Subtract to 80, 70, so on every time
ReleaseSol_ON
		bsf			LATD, Sol
		call        Delay5us
        decfsz      Counter
        goto        ReleaseSol_ON
        movlf       0, Counter                      ; Start with 10% off
        movf        Counter2, W
        addwf       Counter                         ; Add to 20, 30 and so on
ReleaseSol_OFF
        bcf         LATD, Sol
        call        Delay5us
        decfsz      Counter
        goto        ReleaseSol_OFF
        goto        ReleaseSol_START
END_ReleaseSol
            endm


PullSol  macro      Sol, Delay
        local       PullSol_START, PullSol_ON, PullSol_OFF, END_PullSol
        ; Use gradual PWM to pull ArmSol slowly
        movlw       Delay
        movwf       delayReg             ; Say 10 ^ 2 ms to turn it off
        clrf        Counter2
PullSol_START
        incf        Counter2
        dcfsnz      delayReg
        goto        END_PullSol
        movlf       Delay, Counter              ; Start with 90% off
        movf        Counter2, W
        subwf       Counter                     ; Subtract to 80, 70, so on every time
PullSol_OFF
		bcf			LATD, Sol
		call        Delay5us
        decfsz      Counter
        goto        PullSol_OFF
        movlf       0, Counter                  ; Start with 10% on
        movf        Counter2, W
        addwf       Counter                     ; Add to 20, 30 and so on
PullSol_ON
        bsf         LATD, Sol
        call        Delay5us
        decfsz      Counter
        goto        PullSol_ON
        goto        PullSol_START
END_PullSol
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

; ****************************************************************************
; INTERRUPT SERVICE ROUTINE
; ****************************************************************************

ISR_HIGH
	saveContext								; 3
	; Reset Timer
	movlw		TimerConstantH				; 1
	movwf		TMR0H						; 1
	movlw		TimerConstantL - 9			; 1
	movwf		TMR0L						; 1
	; TIMER INTERRUPT
	btfss		INTCON, TMR0IF
	goto		END_ISR_HIGH
	; Increment as BCD interrupts and seconds
	incf_BCD	Op_Interrupts
	movlw		d'0'
	cpfseq		Op_Interrupts				; Skip to seconds++ if interrupts is 0 after adding (100)
	goto		END_ISR_HIGH				; Skip seconds++ if interrupts > 0 after adding (!= 100)
	incf_BCD	Op_Seconds					
END_ISR_HIGH
	bcf			INTCON, TMR0IF
	restContext
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
SavingData		db	"Saving Data...", 0
OpLog_L1		db	"Op. Time: ", 0
OpLog_L2		db	"12:00PM, 2/3/14", 0
OpLogDetails_L1	db	"#: 123 456 789", 0
OpLogDetails_L2	db	"?: ", 0
PermLogMenu_L1	db	"Permanent Logs", 0
PermLogMenu_L2	db	"1 ~ 9", 0
PermLog_L1		db	"P. Log ", 0
PCInter_L1		db	"PC Interface", 0
PCInter_L2		db	"0: Start", 0
PCTransfer_L1	db	"Transfering...", 0
PCLog_Intro     db  "Log Time and Date: ", 0
PCInter_OpTime  db  "Operation Speed and Date: ", 0
JAMMED_L1       db  "MACHINE IS JAMMED", 0
NoData			db	"N/A", 0

; ****************************************************************************
; MAIN PROGRAM
; ****************************************************************************
        CODE
Init
		; Setup I/O
        movlw       b'00000000'
        movwf       TRISA
        movlw		b'11111111'		; Set required keypad inputs (RB1 is interrupt)
        movwf		TRISB
		movlw		b'10111111'		; RC7: USART RC, RC6: USART TX
		movwf		TRISC			; RC3, RC4: for RTC
		movlw		b'00000000'
		movwf		TRISD
        movlw       b'00000111'
        movwf       TRISE
        movlw       b'00001111'     ; Set all AN pins to Digital
        movwf       ADCON1
		; Clear Ports
        clrf		LATA
        clrf		LATB
        clrf		LATC
        clrf		LATD
		clrf		LATE
		; Setup Peripherals
		call		InitLCD
		call		i2c_common_setup
		call		InitUSART
		movlw		b'00001000'			; No 16-bit, internal, no prescaler
		movwf		T0CON
		; Setup Interrupts
		clrf		RCON
		clrf		INTCON
		clrf		INTCON2
		clrf		INTCON3
		bsf         RCON, IPEN          ; Priority mode interrupts
        bsf         INTCON, GIEH        ; Allow global interrupt
		bsf			INTCON, GIEL
        bsf         INTCON2, INTEDG1    ; set INTEDG1 to detect rising edge
		bsf			INTCON, TMR0IE
		bsf			INTCON2, TMR0IP		; set TRM0IP to high priority
        bsf         INTCON3, INT1IE     ; enable INT1 int flag bit on RB1
		bcf			INTCON3, INT1IP		; set INT1IP to low priority
		; Clear FSR
		clrf		EEPROM_H			; Initialize EEPROM address
		clrf		EEPROM_L
		clrf		Counter
		clrf		FL_count
		clrf		TrayEncoder
		clrf		LED_Count
		clrf		SkipCount
		clrf		InOperation
		clrf		Op_Seconds
		clrf		Op_Interrupts
		clrf		tens_digit
		clrf		ones_digit
		call		ClearEEPROM_21

		; SET UP RTC            11:30PM, 02/19/2014
;		rtc_resetAll
;		rtc_set 0x00, b'00000000'		; Set seconds to 0
;		rtc_set 0x01, b'00001000'		; Set minutes (30)
;		rtc_set	0x02, b'01100011'		; Set hours (11PM) (0, 12hour/24hour, PM/AM, 10hour)
;		rtc_set 0x04, b'00010110'		; Set day (19)
;		rtc_set	0x05, b'00000011'		; Set month (2)
;		rtc_set 0x06, b'00010100'		; Set year (14)


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
		movlw		TimerConstantH				; 1
		movwf		TMR0H
		movlw		TimerConstantL				; 1
		movwf		TMR0L						; 1
		bsf			T0CON, TMR0ON				; Turn on timer
		call		ClearEEPROM_21
		; Display
        setf        InOperation
		call		ClrLCD
		DispTable	Operation_L1
		call		LCD_L2
		DispTable	Operation_L2
		; Initialize variables for new operation
		clrf		FL_count
		clrf		Op_Seconds
		clrf		Op_Interrupts
		clrf		TrayEncoder
        clrf        StepCounter
        incf        StepCounter
;FIND_FIRST_FL                                   ; Keep rotating until break beam is (1) not broken
;        btfss       BreakBeam
;        goto        FIND_FL
;		XDegreeStep 1
;        goto        FIND_FIRST_FL

FIND_FL
        clrf        LATC
        btfss       BreakBeam                   ; Skip if beam is not broken (1)
        goto        FOUND_FL                    ; If beam is broken (0) go to FOUND_FL
		movlw		MaxTrayStep
        cpfslt      TrayEncoder                 ; Skip if TrayEncoder < 50
        goto        NO_MORE_FL                  ; If TrayEncoder == 50 go to NO_MORE_FL
		XDegreeStep 1
        goto        FIND_FL

FOUND_FL
        clrf        TrayEncoder                 ; Reset tray encoder since FL was found
        incf		FL_count
        ; TURN ON LED
        Delay50xNms delayReg, OpDelay
        bsf         LATD, ArmSol
		Delay50xNms	delayReg, OpDelay			; Delay
        bsf         LATD, GripSol
		Delay50xNms	delayReg, OpDelay			; Delay
        call        TurnGripCW                  ; Turn grip CW
        ; TAKE DATA FROM PHOTO SENSORS
        call        PhotoData
		Delay50xNms	delayReg, OpDelay			; Delay

;        ; Check if FL was '3 FAIL' and try to turn FL on again if that was the case
;        movlw       b'00001000'
;        cpfsgt      PhotoInput
;        goto        ON_AGAIN                    ; Skip if any LED was on
;        goto        TURN_OFF_FL
;ON_AGAIN
;       ReleaseSol  GripSol, GripSolDelay
;		Delay50xNms	delayReg, OpDelay			; Delay
;       ReleaseSol  ArmSol, ArmSolDelay
;		Delay50xNms	delayReg, OpDelay			; Delay
;        call        TurnGripCCW
;		Delay50xNms	delayReg, OpDelay			; Delay
;       PullSol     ArmSol, ArmSolDelay
;		Delay50xNms	delayReg, OpDelay			; Delay
;       PullSol     GripSol, GripSolDelay
;		Delay50xNms	delayReg, OpDelay			; Delay
;        call        TurnGripCW
;		Delay50xNms	delayReg, OpDelay			; Delay
;        call        RePhotoData                ; Retake PhotoData once before turning off
;TURN_OFF_FL

;        ReleaseSol  GripSol, GripSolDelay
;        Delay50xNms delayReg, OpDelay
;        PullSol     GripSol, GripSolDelay
;        Delay50xNms delayReg, OpDelay

        call        TurnGripCCW                  ; Turn grip CCW
		Delay50xNms	delayReg, OpDelay			 ; Delay
        ReleaseSol  GripSol, GripSolDelay
		Delay50xNms	delayReg, OpDelay   		 ; Delay
        ReleaseSol  ArmSol, ArmSolDelay
		Delay50xNms	delayReg, OpDelay
;        clrf        Counter
;OFF_AGAIN
;        ; CHECK IF FL IS OFF, or tried to turn off 2 more times already
;        movlw       d'2'
;        cpfslt      Counter
;        goto        FL_IS_OFF                   ; Skip if Counter is 2
;        btfsc       Photo3
;        goto        FL_IS_OFF
;        btfsc       Photo2
;        goto        FL_IS_OFF
;        btfsc       Photo1
;        goto        FL_IS_OFF
;        ; IF NOT TRY TURNING OFF AGAIN
;        call        TurnGripCW
;        Delay50xNms delayReg, OpDelay
;        PullSol     ArmSol, ArmSolDelay
;        Delay50xNms delayReg, OpDelay
;        PullSol     GripSol, GripSolDelay
;        Delay50xNms delayReg, OpDelay
;        call        TurnGripCCW
;        Delay50xNms delayReg, OpDelay
;        ReleaseSol  GripSol, GripSolDelay
;        Delay50xNms delayReg, OpDelay
;        ReleaseSol  ArmSol, ArmSolDelay
;        Delay50xNms delayReg, OpDelay
;        incf        Counter
;        goto        OFF_AGAIN
;FL_IS_OFF
;        clrf        Counter
        ; CHECK EXIT CONDITIONS
        movlw       9                           ; Don't exit if under 9 count
        cpfslt      FL_count					; Skip if less than 9 FL counted
        goto        EXIT_OP                     ; Exit if FL_count is 9
		XDegreeStep	TrayStep

        btfss       BreakBeam                                                                           ; NEW CODE
        goto        JAMMED

        goto        FIND_FL                     ; Keep looking for FL if under 9 count

JAMMED
        call        ClrLCD
        DispTable   JAMMED
        call        LCD_L2
        DispTable   Operation_L2


NO_MORE_FL
        incf		FL_count
        call        NoPhotoData					; Take data for N/A FL
        movlw       9                           ; Don't exit if under 9 count
        cpfslt      FL_count
        goto        EXIT_OP                     ; Exit if FL_count is 9
		goto		NO_MORE_FL

EXIT_OP
		;Stop Timer
		bcf			T0CON, TMR0ON
		WriteEEPROM	Op_Seconds,	EEPROM_H, EEPROM_L
		incf		EEPROM_L
		WriteEEPROM	Op_Interrupts, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		; Take RTC Data
		rtc_read	0x02						; Hours
		call		WriteEEPROM_RTC
		rtc_read	0x01						; Minutes
		call		WriteEEPROM_RTC
		rtc_read	0x05						; Month
		call		WriteEEPROM_RTC
		rtc_read	0x04						; Day
		call		WriteEEPROM_RTC
		rtc_read	0x06						; Year
		call		WriteEEPROM_RTC
		; Clear InOperation flag to stop '*' interrupts
		clrf		InOperation
        clrf        LATA                        ; Demagnetize stepper
        goto        SaveData

; SAVE DATA STATE
SaveData
		call		ClrLCD
		DispTable	SavingData
		movlf		d'0', EEPROM_H
		movlf		LastPermLog, EEPROM_L
		movlf		d'0', TempEEPROM_H
		movlf		TempEEPROM_L_Const, TempEEPROM_L
		clrf		Counter2
		clrf		Counter
ShiftLoop
		incf		Counter
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movlw		d'21'
		addwf		EEPROM_L
		WriteEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movlw		d'20'
		subwf		EEPROM_L
		movlw		d'21'
		cpfseq		Counter
		goto		ShiftLoop
	; Move a run (21 bytes) to TempEEPROM
;SaveTempLoop
;		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
;		WriteEEPROM	EEPROM_REG, TempEEPROM_H, TempEEPROM_L
;		incf		EEPROM_L
;		incf		TempEEPROM_L
;		incf		Counter
;		movlw		d'21'
;		cpfseq		Counter						; Keep looping until all 21 bytes are moved to Temp
;		goto		SaveTempLoop
;	; Move TempEEPROM to the next 21 byte block in EEPROM run data
;		movlf		TempEEPROM_L_Const, TempEEPROM_L
;		clrf		Counter
;TempShiftLoop
;		ReadEEPROM	EEPROM_REG, TempEEPROM_H, TempEEPROM_L
;		WriteEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
;		incf		EEPROM_L
;		incf		TempEEPROM_L
;		incf		Counter
;		movlw		d'21'
;		cpfseq		Counter
;		goto		TempShiftLoop
	; Set EEPROM address to the previous 21 byte block, and reset TempEEPROM address
		movlw		d'42'
		subwf		EEPROM_L
		movlf		TempEEPROM_L_Const, TempEEPROM_L
		clrf		Counter
		incf		Counter2
		movlw		d'9'
		cpfseq		Counter2					; Skip if 9 shifts were made
		goto		ShiftLoop
	; Finish Saving Data
		goto		OpLog
		
; OPERATION LOG STATE
OpLog
		call		ClrLCD
		DispTable	OpLog_L1
		; Display Op Time for most recent run
		movlw		d'9'
		movwf		EEPROM_L
		ReadEEPROM	Op_Seconds, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		ReadEEPROM	Op_Interrupts, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		call		DispOpTime
		; Display Op RTC
		call		LCD_L2
		clrf		EEPROM_L
		call		DispOpRTC
Stay_OpLog
		call		ReadKEY
		ChangeState	key0, OpLogDetails
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
		DispTable	PermLogMenu_L1
		call LCD_L2
		DispTable	PermLogMenu_L2
Stay_PermLogMenu
		call		ReadKEY
		ChangeState	key1, PermLog
		ChangeState	key2, PermLog
		ChangeState	key3, PermLog
		ChangeState	key4, PermLog
		ChangeState	key5, PermLog
		ChangeState	key6, PermLog
		ChangeState	key7, PermLog
		ChangeState	key8, PermLog
		ChangeState	key9, PermLog
		ChangeState keyStar, Standby
		bra			Stay_PermLogMenu
PermLog
		call		ClrLCD
		DispTable	PermLog_L1
		; Check if * was pressed from details, which means skip N calculation
		movlw		keyStar
		cpfslt		KEY									; Skip if 1~9 was pressed
		goto		BackFromDetails
		; Find N, the actual key number
		movff		KEY, WREG
		incf		WREG								; W = KEY+1
		movwf		Temp								; Temp = KEY+1
		rrncf		WREG								; W = (KEY+1) / 4
		bcf			WREG, 7
		rrncf		WREG
		bcf			WREG, 7
		subwf		Temp								; N = Temp = (KEY+1) - (KEY+1) / 4
		movff		Temp, WREG							; W = N
		movff		Temp, TempKEY
		mullw		d'21'								; PRODL = 21 x N (No PRODH since N can go up to 9 only)
BackFromDetails
		; Display N
		movff		TempKEY, WREG
		addlw		0x30
		call		WR_DATA
		movlw		":"
		call		WR_DATA
		movlw		" "
		call		WR_DATA
		; Display OpTime
		movff		PRODL, EEPROM_L						; Set EEPROM to N x 21
		movlw		d'9'
		addwf		EEPROM_L
		ReadEEPROM	Op_Seconds, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		ReadEEPROM	Op_Interrupts, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		call		DispOpTime
		movff		PRODL, EEPROM_L						; Reset EEPROM to beginning of run (N x 21)
		; Display OpRTC
		call		LCD_L2
		movff		PRODL, EEPROM_L
		call		DispOpRTC
Stay_PermLog
		call		ReadKEY
		ChangeState	key0, PermLogDetails
		ChangeState keyStar, PermLogMenu
		bra			Stay_PermLog
PermLogDetails
		call		ClrLCD
		DispTable	OpLogDetails_L1
		call        LCD_L2
		DispTable	OpLogDetails_L2
		movff		PRODL, EEPROM_L						; Reset EEPROM to beginning of run
		DispOpLog	EEPROM_H, EEPROM_L
Stay_PermLogDetails
		call		ReadKEY
		ChangeState keyStar, PermLog
		bra			Stay_PermLogDetails

; PC INTERFACE STATE
PCInter
		movff		PCL, PCInter_PCL					; Save PCL for '*' interrupt
		movff		PCLATH, PCInter_PCLATH
		movff		PCLATU,	PCInter_PCLATU
		call		ClrLCD
		DispTable	PCInter_L1
		call		LCD_L2
		DispTable	PCInter_L2
Stay_PCInter
		call		ReadKEY
		ChangeState	key0, PCTransfer
		ChangeState	keyStar, Standby
		bra			Stay_PCInter
PCTransfer
		setf		InTransfer							; Set InTransfer flag for '*' interrupt
		call		ClrLCD
		DispTable	PCTransfer_L1
		call		LCD_L2
		DispTable	Operation_L2
		call		SendDataUSART
		clrf		InTransfer
		bra			PCInter


; ****************************************************************************
; SUBROUTINES
; ****************************************************************************

; DELAYS
Delay5us                            ; Delay 10 cycles
        movlw       SolDelay        ; 1
        movwf       reg5us          ; 1
loop5us
        decfsz      reg5us          ; (3 * 3) - 1 = 5 cycles
        goto        loop5us
        nop                         ; 1
        return                      ; 2

Delay200us							; delay 497 cycles (10MHz Clock)
		movlw		0xA4			; 1 cycle
		movwf		reg200us		; 1 cycle
loop200us
		decfsz		reg200us		; (3 * 164) - 1 = 491 cycles
		goto		loop200us
		nop							; 1 cycle
		nop							; 1 cycle
		return						; 2 cycles

Delay50ms							; delay 125,000 cycles (10MHz Clock)
		movlw		0xF9			; 1 cycle
		movwf		reg50ms			; 1 cycle
loop50ms
		 call		Delay200us
		 decfsz		reg50ms			; (497 + 3) * 249 - 1 = 124,499 Cycles
		 goto		loop50ms
		 call		Delay200us		; 497 cycles
		 return						; 2 cycles

; DISPLAY SUBROUTINES
DispOpTime
		; Display 'No Data' if there was no stored time
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movlw		0xFF
		cpfseq		EEPROM_REG
		goto		NoSkipDispOpTime
		DispTable	NoData
		movlw		0xFF
		cpfslt		EEPROM_REG
		goto		SkipDispOpTime
NoSkipDispOpTime
		; 10's seconds
		swapf		Op_Seconds, W				; Swap and mask upper nibble of Op_Seconds
		movwf		Temp
		movlw		0x0F
		andwf		Temp						; Temp = upper nibble of Op_Seconds
		movff		Temp, WREG					; W = Temp
		addlw		0x30						; Convert to ASCII
		call		WR_DATA
		; 1's seconds
		movff		Op_Seconds, Temp			; Mask lower nibble of Op_Seconds
		movlw		0x0F
		andwf		Temp						; Temp = lower nibble of Op_Seconds
		movff		Temp, WREG					; W = Temp
		addlw		0x30						; Convert to ASCII
		call		WR_DATA
		; Write '.'
		movlw		0x2E
		call		WR_DATA
		; .1's seconds
		swapf		Op_Interrupts, W			; Swap and mask upper nibble of Op_Interrupts
		movwf		Temp
		movlw		0x0F
		andwf		Temp						; Temp = upper nibble of Op_Interrupts
		movff		Temp, WREG					; W = Temp
		addlw		0x30						; Convert to ASCII
		call		WR_DATA
		; Write 's'
		movlw		0x73
		call		WR_DATA
		call		LCD_L2
SkipDispOpTime
		return

DispOpRTC
		; Initiate EEPROM_L
		movlw		d'11'
		addwf		EEPROM_L
		; Display 'No Data' if there was no stored time
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movlw		0xFF
		cpfseq		EEPROM_REG
		goto		NoSkipDispOpRTC
		DispTable	NoData
		movlw		0xFF
		cpfslt		EEPROM_REG
		goto		SkipDispOpRTC
NoSkipDispOpRTC
		; Hour
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
        andlw       b'11110001'
		call		WR_DATA
		incf		EEPROM_L
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
		call		WR_DATA
		incf		EEPROM_L
		; Print ':'
		movlw		":"
		call		WR_DATA
		; Minute
		call DispOpRTC_Helper
        ; AM/PM
        movlw       d'4'
        subwf       EEPROM_L
        ReadEEPROM  EEPROM_REG, EEPROM_H, EEPROM_L
        movlw       "P"
        btfss       EEPROM_REG, 1
        movlw       "A"
        call        WR_DATA
        movlw       "M"
        call        WR_DATA
        movlw       d'4'
        addwf       EEPROM_L
		; Print ' '
		movlw		" "
		call		WR_DATA
		; Month
		call DispOpRTC_Helper
		; Print '/'
		movlw		"/"
		call		WR_DATA
		; Day
		call DispOpRTC_Helper
		; Print '/'
		movlw		"/"
		call		WR_DATA
		call DispOpRTC_Helper
SkipDispOpRTC
		return
DispOpRTC_Helper
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
		call		WR_DATA
		incf		EEPROM_L
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
		call		WR_DATA
		incf		EEPROM_L
		return

; DATA SUBROUTINES
PhotoData                                           ; CALLED FROM FOUND_FL
        movff       PORTC, PhotoInput               ; Take input from photo sensors
        bsf         PhotoInput, 3                   ; Set 1 for break beam since FL was found           ; FIX RC3 for this
        movlw       b'00001111'
        andwf       PhotoInput
        WriteEEPROM PhotoInput, EEPROM_H, EEPROM_L
        incf		EEPROM_L
        return

RePhotoData
        movff       PORTC, PhotoInput
        bsf         PhotoInput, 3
        movlw       b'00001111'
        andwf       PhotoInput
        decf        EEPROM_L
        WriteEEPROM PhotoInput, EEPROM_H, EEPROM_L
        incf        EEPROM_L
        return

NoPhotoData
        clrf        PhotoInput
        WriteEEPROM PhotoInput, EEPROM_H, EEPROM_L
        incf		EEPROM_L
        return

WriteEEPROM_RTC
		WriteEEPROM	tens_digit, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		WriteEEPROM ones_digit, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		return

ClearEEPROM_21
		clrf		EEPROM_H
		clrf		EEPROM_L
		setf		EEPROM_CLEAR
		clrf		Counter
ClearNext
		WriteEEPROM EEPROM_CLEAR, EEPROM_H, EEPROM_L
		incf		Counter
		incf		EEPROM_L
		movlw		d'21'
		cpfseq		Counter
		goto		ClearNext
		clrf		EEPROM_H				; reset EEPROMaddr
		clrf		EEPROM_L
		return

; ACTUATOR SUBROUTINES
Step1
        bsf         Step1a
        bcf         Step1b
        bcf         Step2a
        bsf         Step2b
        call        StepDelay
        return
Step2
        bsf         Step1a
        bcf         Step1b
        bsf         Step2a
        bcf         Step2b
        call        StepDelay
        return
Step3
        bcf         Step1a
        bsf         Step1b
        bsf         Step2a
        bcf         Step2b
        call        StepDelay
        return
Step4
        bcf         Step1a
        bsf         Step1b
        bcf         Step2a
        bsf         Step2b
        call        StepDelay
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
        movlw       1
        cpfseq      StepCounter
        goto        SkipStep1
        call        Step1
        goto        IncStep
SkipStep1
        movlw       2
        cpfseq      StepCounter
        goto        SkipStep2
        call        Step2
        goto        IncStep
SkipStep2
        movlw       3
        cpfseq      StepCounter
        goto        SkipStep3
        call        Step3
        goto        IncStep
SkipStep3
        call        Step4
IncStep
        incf        StepCounter
        movlw       5
        cpfseq      StepCounter
        goto        EndStep
        goto        ResetStep
ResetStep
        clrf        StepCounter
        incf        StepCounter
EndStep
		return

TurnGripCW
        movlf       GripMotorDelay, delayReg
TurnGripCW_START
        dcfsnz      delayReg
        goto        END_TurnGripCW
        movlf       GripMotorOn, Counter
TurnGripCW_ON
		bsf			GripMotorCW
		call        Delay200us
        decfsz      Counter
        goto        TurnGripCW_ON
        movlf       GripMotorOff, Counter
TurnGripCW_OFF
        bcf         GripMotorCW
        call        Delay200us
        decfsz      Counter
        goto        TurnGripCW_OFF
        goto        TurnGripCW_START
END_TurnGripCW
        return

TurnGripCCW
        movlf       GripMotorDelay, delayReg
TurnGripCCW_START
        dcfsnz      delayReg
        goto        END_TurnGripCCW
        movlf       GripMotorOn, Counter
TurnGripCCW_ON
		bsf			GripMotorCCW
		call        Delay200us
        decfsz      Counter
        goto        TurnGripCCW_ON
        movlf       GripMotorOff, Counter
TurnGripCCW_OFF
        bcf         GripMotorCCW
        call        Delay200us
        decfsz      Counter
        goto        TurnGripCCW_OFF
        goto        TurnGripCCW_START
END_TurnGripCCW
        return

ReleaseArmSol
        ; Use gradual PWM to release ArmSol slowly
        movlf       ArmSolDelay, delayReg           ; Say 10 ^ 2 ms to turn it off
        clrf        Counter2
ReleaseArmSol_START
        incf        Counter2
        dcfsnz      delayReg
        goto        END_ReleaseArmSol
        movlf       ArmSolDelay, Counter            ; Start with 90% on
        movf        Counter2, W
        subwf       Counter                         ; Subtract to 80, 70, so on every time
ReleaseArmSol_ON
		bsf			LATD, ArmSol
		call        Delay200us
        decfsz      Counter
        goto        ReleaseArmSol_ON
        movlf       0, Counter                      ; Start with 10% off
        movf        Counter2, W
        addwf       Counter                         ; Add to 20, 30 and so on
ReleaseArmSol_OFF
        bcf         LATD, ArmSol
        call        Delay200us
        decfsz      Counter
        goto        ReleaseArmSol_OFF
        goto        ReleaseArmSol_START
END_ReleaseArmSol
        return

ReleaseGripSol
        ; Use gradual PWM to release GripSol slowly
        movlf       GripSolDelay, delayReg           ; Say 10 ^ 2 ms to turn it off
        clrf        Counter2
ReleaseGripSol_START
        incf        Counter2
        dcfsnz      delayReg
        goto        END_ReleaseGripSol
        movlf       GripSolDelay, Counter            ; Start with 90% on
        movf        Counter2, W
        subwf       Counter                         ; Subtract to 80, 70, so on every time
ReleaseGripSol_ON
		bsf			LATD, GripSol
		call        Delay200us
        decfsz      Counter
        goto        ReleaseGripSol_ON
        movlf       0, Counter                      ; Start with 10% off
        movf        Counter2, W
        addwf       Counter                         ; Add to 20, 30 and so on
ReleaseGripSol_OFF
        bcf         LATD, GripSol
        call        Delay200us
        decfsz      Counter
        goto        ReleaseGripSol_OFF
        goto        ReleaseGripSol_START
END_ReleaseGripSol
        return


; Read Keypad Input
ReadKEY
WaitKey
		btfss		PORTB,1     ;Wait until data is available from the keypad
        goto		WaitKey		;Once a key is pressed,
        swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F		;Mask 00001111
		movwf		KEY			;Save result in KEY
		btfsc		PORTB,1     ;Wait until key is released
        goto		$-2			;Back 1 instruction
		return

Read_KEY_RTC
WaitKeyRTC
		; Reset to second line
		call		LCD_L2
		; Display hours
		rtc_read	0x02
		movf        tens_digit, W
        andlw       b'00000001'
        addlw       0x30
        call        WR_DATA
        movf        ones_digit, W
        call        WR_DATA
		movlw		":"
		call		WR_DATA
		; Dispay minutes
		rtc_read	0x01
		WriteRTC
		; Dispay AM/PM
		rtc_read	0x02
        movlw       "P"
        btfss       tens_digit, 1
        movlw       "A"
        call        WR_DATA
        movlw       "M"
        call        WR_DATA
		movlw		" "
        call        WR_DATA
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
		rtc_read	0x06
		WriteRTC
		;Process KEY
		btfss		PORTB,1     ;Wait until data is available from the keypad
        goto		WaitKeyRTC	;Once a key is pressed,
        swapf		PORTB,W     ;Read PortB<7:4> into W<3:0>
		andlw		0x0F		;Mask 00001111
		movwf		KEY			;Save result in KEY
		btfsc		PORTB,1     ;Wait until key is released
        goto		$-2			;Back 1 instruction
		return

; USART SUBROUTINES
InitUSART
		MOVLW	15				;Baud rate 9600, assuming 10MHz oscillator
		MOVWF	SPBRG
		clrf	TXSTA			;8 bits data, no parity, 1 stop

		CLRF	RCSTA
		BSF		RCSTA,SPEN		;enable single receive
		BSF		RCSTA,CREN		;continuous

		BSF		TXSTA,TXEN		;enable tx
		return

SendDataUSART

;ERASE
;        movlf       255, Counter
;        movlw       0x08
;        call        TransmitWaitUSART
;        decfsz      Counter
;        goto        ERASE
;        movlf       255, Counter2
;        decfsz      Counter2
;        goto        ERASE

		movlf		d'21', EEPROM_L         ; Set EEPROM_L to beginning of Permanent Log
    ; Clear console - ESC[2J (VT100 escape sequence)
;        movlw       0x1B                    ; ESC
;        call        TransmitWaitUSART
;        movlw       0x5B                    ; [
;        call        TransmitWaitUSART
;        movlw       0x32                    ; 2
;        call        TransmitWaitUSART
;        movlw       0x4A                    ; J
;        call        TransmitWaitUSART
;        movlw       0x02
;        call        TransmitWaitUSART
    ; Write Transfer Time/Date
		SendTable   PCLog_Intro
    ; RTC display
		; Display hours
		rtc_read	0x02
		movf        tens_digit, W
        andlw       b'00000001'
        addlw       0x30
        call        TransmitWaitUSART
        movf        ones_digit, W
        call        TransmitWaitUSART
		movlw		":"
		call		TransmitWaitUSART
		; Dispay minutes
		rtc_read	0x01
		call        SendRTC_USART
		; Dispay AM/PM
		rtc_read	0x02
        movlw       "P"
        btfss       tens_digit, 1
        movlw       "A"
        call        TransmitWaitUSART
        movlw       "M"
        call        TransmitWaitUSART
		movlw		" "
        call        TransmitWaitUSART
		; Display month
		rtc_read	0x05
		call        SendRTC_USART
		movlw		0x2F		; ASCII '/'
		call		TransmitWaitUSART
		; Display day
		rtc_read	0x04
		call        SendRTC_USART
		movlw		0x2F		; ASCII '/'
		call		TransmitWaitUSART
		; Display year
		rtc_read	0x06
		call        SendRTC_USART
        ; Newline
        call        NewLineUSART
        call        NewLineUSART
        ; Initialize N = 1
        clrf        Counter
        incf        Counter
SendUSARTLoop
		; Send Op Time
        ;SendTable   PCInter_OpTime
        ;call        NewLineUSART
        movlw       d'21'
        mulwf       Counter
		movff		PRODL, EEPROM_L                     ; EEPROM points to N x 21
		movlw		d'9'
        addwf       EEPROM_L
		ReadEEPROM	Op_Seconds, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		ReadEEPROM	Op_Interrupts, EEPROM_H, EEPROM_L
		incf		EEPROM_L
		call		SendOpTime
		; Send Op RTC
        movlw       0x09                                ; TAB
        call        TransmitWaitUSART
		movff       PRODL, EEPROM_L                     ; Reset EEPROM_L
		call		SendOpRTC
        call        NewLineUSART
        ; Send Op Log
        SendTable   OpLogDetails_L1
        call        NewLineUSART
        SendTable   OpLogDetails_L2
        movff       PRODL, EEPROM_L                     ; Reset EEPROM_L
		SendOpLog	EEPROM_H, EEPROM_L
        call        NewLineUSART
        call        NewLineUSART
        ; Increment counters
        incf        Counter
        ; Check ending conditions
		movlw		6
		cpfseq		Counter
		goto		SendUSARTLoop
		return

TransmitWaitUSART
        movwf		TXREG			;Send it over RS232
        btfss		TXSTA,1        ; check TRMT bit in TXSTA (FSR) until TRMT=1
        goto		$-2
        return


SendRTC_USART
		movff		tens_digit, WREG
		call		TransmitWaitUSART
		movff		ones_digit, WREG
		call		TransmitWaitUSART
        return

NewLineUSART
        movlw       0x0A
        call        TransmitWaitUSART
        movlw       0x0D
        call        TransmitWaitUSART
        return

; Subroutines to Send Run Data to USART
SendOpTime
		; Send 'No Data' if there was no stored time
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movlw		0xFF
		cpfseq		EEPROM_REG
		goto		NoSkipSendOpTime
		SendTable	NoData
		movlw		0xFF
		cpfslt		EEPROM_REG
		goto		SkipSendOpTime
NoSkipSendOpTime
		; 10's seconds
		swapf		Op_Seconds, W				; Swap and mask upper nibble of Op_Seconds
		movwf		Temp
		movlw		0x0F
		andwf		Temp						; Temp = upper nibble of Op_Seconds
		movff		Temp, WREG					; W = Temp
		addlw		0x30						; Convert to ASCII
		call		TransmitWaitUSART
		; 1's seconds
		movff		Op_Seconds, Temp			; Mask lower nibble of Op_Seconds
		movlw		0x0F
		andwf		Temp						; Temp = lower nibble of Op_Seconds
		movff		Temp, WREG					; W = Temp
		addlw		0x30						; Convert to ASCII
		call		TransmitWaitUSART
		; Write '.'
		movlw		0x2E
		call		TransmitWaitUSART
		; .1's seconds
		swapf		Op_Interrupts, W			; Swap and mask upper nibble of Op_Interrupts
		movwf		Temp
		movlw		0x0F
		andwf		Temp						; Temp = upper nibble of Op_Interrupts
		movff		Temp, WREG					; W = Temp
		addlw		0x30						; Convert to ASCII
		call		TransmitWaitUSART
		; Write 's'
		movlw		0x73
		call		TransmitWaitUSART
SkipSendOpTime
		return

SendOpRTC
		; Initiate EEPROM_L
		movlw		d'11'
		addwf		EEPROM_L
		; Send 'No Data' if there was no stored time
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movlw		0xFF
		cpfseq		EEPROM_REG
		goto		NoSkipSendOpRTC
		SendTable	NoData
		movlw		0xFF
		cpfslt		EEPROM_REG
		goto		SkipSendOpRTC
NoSkipSendOpRTC
		; Hour
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
        andlw       b'11110001'
		call		TransmitWaitUSART
		incf		EEPROM_L
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
		call		TransmitWaitUSART
		incf		EEPROM_L
		; Print ':'
		movlw		":"
		call		TransmitWaitUSART
		; Minute
		call SendOpRTC_Helper
        ; AM/PM
        movlw       d'4'
        subwf       EEPROM_L
        ReadEEPROM  EEPROM_REG, EEPROM_H, EEPROM_L
        movlw       "P"
        btfss       EEPROM_REG, 1
        movlw       "A"
        call        TransmitWaitUSART
        movlw       "M"
        call        TransmitWaitUSART
        movlw       d'4'
        addwf       EEPROM_L
		; Print ' '
		movlw		" "
		call		TransmitWaitUSART
		; Month
		call SendOpRTC_Helper
		; Print '/'
		movlw		"/"
		call		TransmitWaitUSART
		; Day
		call SendOpRTC_Helper
		; Print '/'
		movlw		"/"
		call		TransmitWaitUSART
		call        SendOpRTC_Helper
SkipSendOpRTC
		return
SendOpRTC_Helper
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
		call		TransmitWaitUSART
		incf		EEPROM_L
		ReadEEPROM	EEPROM_REG, EEPROM_H, EEPROM_L
		movff		EEPROM_REG, WREG
		call		TransmitWaitUSART
		incf		EEPROM_L
		return

	END