    include <p16f877.inc>
	errorlevel	-302
	errorlevel	-305

;global labels

	global	write_rtc,read_rtc,rtc_convert,i2c_common_setup,p2p_write,p2p_read

;Definition and variable declarations;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
        cblock    0x71			;these variable names are for reference only. The following
        dt1			;0x71		 addresses are used for the RTC module
        dt2			;0x72
        ADD			;0x73
        DAT			;0x74
        DOUT		;0x75
        B1			;0x76
		dig10		;0x77
		dig1		;0x78
        endc

;I2C lowest layer macros;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

i2c_common_check_ack	macro	err_address		;If bad ACK bit received, goto err_address
	banksel		SSPCON2
    btfsc       SSPCON2,ACKSTAT
    goto        err_address
	endm

i2c_common_start	macro
;input:		none
;output:	none
;desc:		initiate start conditionon the bus
	banksel     SSPCON2
    bsf         SSPCON2,SEN
    btfsc       SSPCON2,SEN
    goto        $-1
	endm

i2c_common_stop	macro
;input: 	none
;output:	none
;desc:		initiate stop condition on the bus
	banksel     SSPCON2
    bsf         SSPCON2,PEN
    btfsc       SSPCON2,PEN
    goto        $-1
	endm

i2c_common_repeatedstart	macro
;input:		none
;output:	none
;desc:		initiate repeated start on the bus. Usually used for
;			changing direction of SDA without STOP event
	banksel     SSPCON2
    bsf         SSPCON2,RSEN
    btfsc       SSPCON2,RSEN
    goto        $-1
	endm

i2c_common_ack		macro
;input:		none
;output:	none
;desc:		send an acknowledge to slave device
    banksel     SSPCON2
    bcf         SSPCON2,ACKDT
    bsf         SSPCON2,ACKEN
    btfsc       SSPCON2,ACKEN
    goto        $-1
    endm

i2c_common_nack	macro
;input:		none
;output:	none
;desc:		send an not acknowledge to slave device
   banksel     SSPCON2
   bsf         SSPCON2,ACKDT
   bsf         SSPCON2,ACKEN
   btfsc       SSPCON2,ACKEN
   goto        $-1
   endm

i2c_common_write	macro	
;input:		W
;output:	to slave device
;desc:		writes W to SSPBUF and send to slave device. Make sure
;			transmit is finished before continuing
   banksel     SSPBUF
   movwf       SSPBUF
   banksel     SSPSTAT
   btfsc       SSPSTAT,R_W 		;While transmit is in progress, wait
   goto        $-1
   banksel     SSPCON2
   endm

i2c_common_read	macro
;input:		none
;output:	W
;desc:		reads data from slave and saves it in W.
   banksel     SSPCON2
   bsf         SSPCON2,RCEN    ;Begin receiving byte from
   btfsc       SSPCON2,RCEN
   goto        $-1
   banksel     SSPBUF
   movf        SSPBUF,w
   endm
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	code

i2c_common_setup
;input:		none
;output:	none
;desc:		sets up I2C as master device with 100kHz baud rate
	banksel		SSPSTAT
    clrf        SSPSTAT         ;I2C line levels, and clear all flags
    movlw       d'24'         	;100kHz baud rate: 10MHz osc / [4*(24+1)]
	banksel		SSPADD
    movwf       SSPADD          ;RTC only supports 100kHz

    movlw       b'00001000'     ;Config SSP for Master Mode I2C
	banksel		SSPCON
    movwf       SSPCON
    bsf         SSPCON,SSPEN    ;Enable SSP module
    i2c_common_stop        		;Ensure the bus is free
	return

;rtc Algorithms;;;;;;

write_rtc
;input:		address of register in RTC
;output:	none
;Desc:		handles writing data to RTC
        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
        movlw       0xD0        ;DS1307 address | WRITE bit
        i2c_common_write
        i2c_common_check_ack   WR_ERR

        ;Write data to I2C bus (Register Address in RTC)
		banksel		0x73
        movf        0x73,w       ;Set register pointer in RTC
        i2c_common_write
        i2c_common_check_ack   WR_ERR

        ;Write data to I2C bus (Data to be placed in RTC register)
		banksel		0x74
        movf        0x74,w       ;Write data to register in RTC
        i2c_common_write
        i2c_common_check_ack   WR_ERR
        goto        WR_END
WR_ERR
        nop
WR_END  
		i2c_common_stop	;Release the I2C bus
        return

read_rtc
;input:		address of RTC
;output:	DOUT or 0x75
;Desc:		This reads from the selected address of the RTC
;			and saves it into DOUT or address 0x75
        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
        movlw       0xD0        ;DS1307 address | WRITE bit
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        ;Write data to I2C bus (Register Address in RTC)
		banksel		0x73
        movf        0x73,w       ;Set register pointer in RTC
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        ;Re-Select the DS1307 on the bus, in READ mode
        i2c_common_repeatedstart
        movlw       0xD1        ;DS1307 address | READ bit
        i2c_common_write
        i2c_common_check_ack   RD_ERR

        ;Read data from I2C bus (Contents of Register in RTC)
        i2c_common_read
		banksel		0x75
        movwf       0x75
        i2c_common_nack      ;Send acknowledgement of data reception
        
        goto        RD_END

RD_ERR 
        nop
        
        ;Release the I2C bus
RD_END  i2c_common_stop
        return

rtc_convert   
;input:		W
;output:	dig10 (0x77), dig1 (0x78)
;desc:		This subroutine converts the binary number
;			in W into a two digit ASCII number and place
;			each digit into the corresponding registers
;			dig10 or dig1
	banksel	0x76
	movwf   0x76             ; B1 = HHHH LLLL
    swapf   0x76,w           ; W  = LLLL HHHH
    andlw   0x0f           ; Mask upper four bits 0000 HHHH
    addlw   0x30           ; convert to ASCII
    movwf	0x77		   ;saves into 10ths digit

	banksel	0x76
    movf    0x76,w
    andlw   0x0f           ; w  = 0000 LLLL
    addlw   0x30           ; convert to ASCII		
    movwf	0x78	       ; saves into 1s digit
   	return
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;


;pic to pic subroutines;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
p2p_write
        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
        movlw       b'00010000'
        i2c_common_write
        i2c_common_check_ack   W_END 

		banksel	0x70
		movf	0x70, W
        i2c_common_write
        i2c_common_check_ack   W_END 
        goto        W_END
W_END  
		i2c_common_stop	;Release the I2C bus
        return


p2p_read
        ;Select the DS1307 on the bus, in WRITE mode
        i2c_common_start
		movlw       b'00010001'
        i2c_common_write
		i2c_common_check_ack   R_END

        i2c_common_read
		banksel		0x70
        movwf       0x70
        i2c_common_nack      ;Send acknowledgement of data reception
R_END
		i2c_common_stop
        return
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	end