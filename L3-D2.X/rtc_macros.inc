	;External labels
	extern	write_rtc,read_rtc,rtc_convert,i2c_common_setup

;RTC Macros;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

rtc_resetAll	macro
;input:		none
;output:	none
;desc:		Resets all the time keeping registers on the RTC to zero
	banksel		0x74
	clrf		0x74
	banksel		0x73
	clrf		0x73
    call        write_rtc		;Write 0 to Seconds
	banksel		0x73
    incf        0x73   			;Set register address to 1
	call		write_rtc
	banksel		0x73
    incf        0x73   			;Set register address to 2
	call		write_rtc
	banksel		0x73
    incf        0x73   			;Set register address to 3
	call		write_rtc
	banksel		0x73
    incf        0x73   			;Set register address to 4
	call		write_rtc
	banksel		0x73
    incf        0x73   			;Set register address to 5
	call		write_rtc
	banksel		0x73
    incf        0x73   			;Set register address to 6
	call		write_rtc
	endm

rtc_set		macro	addliteral,datliteral
;input:		addliteral: value of address
;			datliteral: value of data
;output:	none
;desc:		loads the data in datliteral into the 
;			address specified by addliteral in the RTC
	banksel	0x73
	movlw	addliteral
	movwf	0x73
	banksel	0x74
	movlw	datliteral
	movwf	0x74	
	call	write_rtc
	endm

rtc_read	macro	addliteral
;input:		addliteral
;output:	0x75, 0x77, 0x78
;desc:		From the selected register in the RTC, read the data
;			and load it into 0x75. 0x75 is also converted into 
;			ASCII characters and the tens digit is placed into
;			0x77 and the ones digit is placed in 0x78
	movlw	addliteral
	banksel	0x73
	movwf	0x73
	call	read_rtc
	banksel	0x75
	movf	0x75,w
	call	rtc_convert   
	endm
