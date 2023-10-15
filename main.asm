;
; temp_hum_sys.asm
;

;***************************************************************************
;*
;* Title: Temperature Humidity System
;* Author: Omar Mohamed
;* Version: 1.0
;* Last updated: 11/12/2022 
;* Target: AVR128DB48
;*
;* DESCRIPTION
;* This program reads measured data from a DHT11 temperature and humidity sensor
;* and displays the temperature or humidity on a seven segment display every two
;* seconds. A pushbutton connected to PE0 can be used to toggle between displaying
;* either the temperature or humidity
;* 
;* VERSION HISTORY
;* 1.0 Original version
;***************************************************************************

.equ PERIOD = 40		;value for 100.0 Hz frequency

.dseg		;start of data segment
	display_mode: .byte 1		;display temperature or humidity
	measured_data: .byte 5		;five byte array of data recieved from DHT11
	bcd_entries: .byte 4		;four byte array to hold bcd values entered
	led_display: .byte 4		;four byte array to hold display segment values
	digit_num: .byte 1			;position number of last digit displayed
.cseg		;start of code segment
reset:
	jmp start			;reset vector executed a power ON

.org TCA0_OVF_vect		;vector for TCA0 overflow interrupt
	jmp tca0_ovf_ISR 
.org PORTE_PORT_vect
	jmp porte_isr		;vector for all PORTE pin change IRQs

start:
;Configure Ports
	ldi r16, 0x00		;load r16 with all 0's
	out VPORTC_DIR, r16		;PORTC - All pins configured as inputs
	ldi r16, 0xFF		;load r16 with all 1's
	out VPORTD_DIR, r16		;PORTD - All pins configured as outputs
	out VPORTA_DIR, r16		;PORTA - All pins configured as outputs
	cbi VPORTE_DIR, 0		;PE0 configured as input
;initialize peripherals
	ldi r16, 0x00		;load r16 with all 0's
	sts digit_num, r16		;initialize digit_num to 00

	ldi YL, LOW(led_display)		;initialize Y-pointer to led_display[0]
	ldi YH, HIGH(bcd_entries)
	ldi r16, 0xFF		;load r16 with all 1's
	ldi r17, 4			;loop-control variable
	
	initial_loop:
		st Y+, r16			;set all bits in led_display array to 1
		dec r17				;decrement loop-control variable
		brne initial_loop		;loop if != 0
	
	ldi r16, 0x00		
	sts display_mode, r16	;initialize to temperature mode

	lds r16, PORTE_PIN0CTRL	;set ISC for PE0 to rising edge
	ori r16, 0x02		;ISC = 2 for rising edge
	sts PORTE_PIN0CTRL, r16

	;configure TCA0
	ldi r16, TCA_SINGLE_WGMODE_NORMAL_gc	;WGMODE normal
	sts TCA0_SINGLE_CTRLB, r16

	ldi r16, TCA_SINGLE_OVF_bm		;enable overflow interrupt
	sts TCA0_SINGLE_INTCTRL, r16

	;load period low byte then high byte
	ldi r16, LOW(PERIOD)		;set the period
	sts TCA0_SINGLE_PER, r16
	ldi r16, HIGH(PERIOD)
	sts TCA0_SINGLE_PER + 1, r16

	;set clock and start timer
	ldi r16, TCA_SINGLE_CLKSEL_DIV256_gc | TCA_SINGLE_ENABLE_bm
	sts TCA0_SINGLE_CTRLA, r16

	sei		;enable global interrupts

main_loop:
	rcall send_start	;send start signal to DHT11
	rcall wait_for_response		;wait for response from DHT11
	rcall get_measured_data		;get measured data from DHT11	
	rcall bin2BCD		;convert measured data to BCD values
	rcall bcd_entries_to_led_display	;store updated display codes in led_display
	rcall round_to_3sig_figs	;only display 3 most significant figures
	rjmp main_loop

;Interrupt service routine for any PORTE pin change IRQ
porte_ISR:
	cli				;clear global interrupt enable, I = 0
	push r16		;save r16 then SREG, note I = 0
	in r16, CPU_SREG
	push r16

	;Determine which pins of PORTE have IRQs
	lds r16, PORTE_INTFLAGS	;check for PE0 IRQ flag set
	sbrs r16, 0
	reti
	
	lds r16, display_mode
	com r16		;switch display mode
	andi r16, 0x01	;mask for lsb
	sts display_mode, r16
	
	;Clear IRQ flag
	ldi r16, 0x01
	sts PORTE_INTFLAGS, r16
	
	pop r16			;restore SREG then r16
	out CPU_SREG, r16	;note I in SREG now = 0
	pop r16
	sei				;SREG I = 1
	reti			;return from PORTE pin change ISR
;Note: reti does not set I on an AVR128DB48


;Interrupt service routine for overflow of TCA0
tca0_ovf_ISR:
	push r16		;save r16 then SREG
	in r16, CPU_SREG
	push r16

	rcall multiplex_display		;multiplex seven-segment display

	ldi r16, TCA_SINGLE_OVF_bm	;clear OVF flag
	sts TCA0_SINGLE_INTFLAGS, r16

	pop r16				;restore registers
	out CPU_SREG, r16
	pop r16

	reti

;***************************************************************************
;* 
;* "bin2BCD" - 8-bit Binary to 2-digit BCD Conversion
;*
;* Description:
;* converts an 8-bit binary value to a 2-digit BCD number. The binary number
;* must not exceed 99, since a 2-digit BCD number can only represent 0 - 99.
;*  
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified: 
;*
;* Parameters:	
;* 
;* Returns:
;* 5 bytes of data from the DHT11 stored in memory in the array measured_data
;* 
;* Notes: 
;* 
;***************************************************************************

bin2BCD:
	cli		;disable global interrupts

	push r16	;store registers
	push r17
	push r18
	push YL
	push YH
	push ZL
	push ZH
;determine which data to display: temperature or humidity
	lds r16, display_mode
	cpi r16, 0x00	;temperature mode?
	breq temp_mode
	cpi r16, 0x01	;humidity mode?
	breq humidity_mode
humidity_mode:
	ldi ZL, LOW(measured_data)		;start at byte 1, humidity integral
	ldi ZH, HIGH(measured_data)
	rjmp next
temp_mode:
	ldi ZL, LOW(measured_data + 2)		;start at byte 3, temperature integral
	ldi ZH, HIGH(measured_data + 2)
next:
	ldi YL, LOW(bcd_entries)	;start at 1 after end of bcd_entries for pre-decrement 
	ldi YH, HIGH(bcd_entries)

	ld r16, Z		;load byte from measured_data into register
	rcall get_MSD	;convert from 8-bit binary to 2-digit BCD  	

	st Y+, r18		;store MSD in bcd_entries
	st Y+, r16		;store LSD in bcd_entries
;increment pointer to measured_data array to decimal byte of data
	adiw Z, 0x01	;move to decimal byte

	ld r16, Z		;load byte from measured_data into register
	rcall get_MSD	;convert from 8-bit binary to 2-digit BCD  	 

	st Y+, r18		;store MSD in bcd_entries
	st Y+, r16		;LSD is left in original register, store in bcd_entries
	
	pop ZH		;restore registers
	pop ZL
	pop YH
	pop YL
	pop r18
	pop r17
	pop r16
	
	sei		;enable global interrupts
	ret

;***************************************************************************
;* 
;* "get_MSD" - Get Most Significant Digit
;*
;* Description:
;* Gets most significan digit in 8-bit binary value between 0-99 and leaves 
;* the least significant digit is left in the input register
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified: r16, r18
;*
;* Parameters:	r16
;* 
;* Returns:
;* Most Significant Digit in r18
;* Least Significant Digit in r16
;* 
;* Notes: 
;* 
;***************************************************************************

get_MSD:
	push r17
	 
	ldi r17, 0x0A	;load 10 into register
	ldi r18, 0x00	;result MSD 
	cpi r16, 0x0A	;chek if there is a 10s digit or not
	brlt return
	find_MSD:
		sub r16, r17		;subtract 10 from original register
		inc r18			;increment result MSD 
		cpi r16, 0x0A	
		brge find_MSD	;repeat until original register is less than 10
return:
	pop r17
	ret

;***************************************************************************
;* 
;* "bcd_entries_to_led_display" - bcd_entries to led_display
;*
;* Description:
;* Transfer contents of bcd_entries array to led_display array in reversed
;* order.  
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified:
;*
;* Parameters:	
;* led_display: a four byte array that holds the segment values
;*  for each digit of the display. led_display[0] holds the segment pattern
;*  for the digit 0 (rightmost digit) and so on.
;* bcd_entries: a four byte array that holds the BCD digit values of the 
;*	four byte array measured_data
;*
;* Returns:
;* led_display array with contents of bcd_entries in reversed order
;* 
;* Notes: 
;* 
;***************************************************************************

bcd_entries_to_led_display:
	cli		;disable global interrupts

	push r17		;store registers
	push r18
	push YL
	push YH
	push ZL
	push ZH

;convert bcd_entries to hex_display codes and place in led_display
	ldi ZL, LOW(bcd_entries)		;initialize Z-pointer to bcd_entries[0]
	ldi ZH, HIGH(bcd_entries)
	ldi YL, LOW(led_display + 4)		;initialize Y-pointer to end of led_display
	ldi YH, HIGH(led_display + 4)
	ldi r17, 0x04		;loop control variable
transfer_data:
	ld r18, Z+			;load bcd_entries[i] in r18 and increment pointer
	rcall hex_to_7seg	;convert hex-values to segment values
	st -Y, r18			;store hex_value of bcd_entries[i] in led_display[i] and increment Y pointer
	dec r17				;decrement loop control variable
	brne transfer_data	;loop 4 times

	pop ZH		;restore registers
	pop ZL
	pop YH
	pop YL
	pop r18
	pop r17

	sei		;enable global interrupts
	ret

;***************************************************************************
;* 
;* "round_to_3sig_figs" - Round to three significant figures
;*
;* Description:
;* Rounds the value on the display to three significant figures by shifting
;* led_display array and turning the leftmost digit OFF

;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-16-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified:
;*
;* Parameters:	
;* led_display: a four byte array that holds the segment values
;*  for each digit of the display. led_display[0] holds the segment pattern
;*  for the digit 0 (rightmost digit) and so on.
;*
;* Returns:
;* led_display array shifted left with last byte containing all 1s
;* 
;* Notes: 
;* 
;***************************************************************************

round_to_3sig_figs:
	push YL			;store registers
	push YH
	push r16
	push r17

	ldi YL, LOW(led_display + 1)		;initialize Y-pointer to led_display[1]
	ldi YH, HIGH(led_display + 1)
	
	ldi r17, 0x03		;loop control variable
;shift display digits right by 1
shift_array:
	ld r16, Y		;store current display code in register
	st -Y, r16		;store value in previous array index
	adiw Y, 0x02	;move to next array index
	dec r17		;decrement loop control variable
	brne shift_array	
	
	ldi YL, LOW(led_display + 3)		;initialize Y-pointer to end of led_display
	ldi YH, HIGH(led_display + 3)
	ldi r16, 0xFF		;turn OFF leftmost digit
	st Y, r16

	pop r17		;restore registers
	pop r16
	pop YH
	pop YL

	ret

;***************************************************************************
;* 
;* "multiplex_display" - Multiplex the Four Digit LED Display
;*
;* Description: Updates a single digit of the display and increments the 
;*  digit_num to the digit position to be displayed next
;* 
;* Author: Omar Mohamed 
;* Version: 1.0
;* Last updated: 10/24/2022 8:07:30 PM
;* Target: AVR128DB48
;* Number of words:
;* Number of cycles:
;* Low registers modified:
;* High registers modified: r30, r31
;*
;* Parameters:
;* led_display: a four byte array that holds the segment values
;*  for each digit of the display. led_display[0] holds the segment pattern
;*  for the digit 0 (rightmost digit) and so on.
;* digit_num: byte variable, the least significant two bits are the index of
;*  the last digit displayed.
;* 
;* Returns: Outputs segment pattern and turns on digit driver for the next 
;*  position in the display to be turned ON. The segment value from one of the
;*  array values will be stored in r31. The value of digit_num will be stored in
;*  r30
;* Notes: The segments are controlled by PORTD (dp, a through g), the digit
;*  drivers are controlled by PORTA (PA7 - PA4, digit 0 - 3).
;*
;***************************************************************************

multiplex_display:
;make sure all digits are off
	sbi VPORTA_OUT, 4		;turn digit 0 OFF
	sbi VPORTA_OUT, 5		;turn digit 1 OFF
	sbi VPORTA_OUT, 6		;turn digit 2 OFF 
	sbi VPORTA_OUT, 7		;turn digit 3 OFF
;read digit_num from memory
	lds r30, digit_num ;store value of digit_num in r30
	andi r30, 0x03 ;mask for least significant two bits
;initialize Y-pointer to led_display[0]
	ldi YL, LOW(led_display)		
	ldi YH, HIGH(led_display) 
;check which digit to update (3-9 clks)
	cpi r30, 0x00		;check if digit_0
	breq digit_0		;output value to digit_0
	cpi r30, 0x01		;check if digit_1
	breq digit_1		;output value to digit_1
	cpi r30, 0x02		;check if digit_2
	breq digit_2		;output value to digit_2
	cpi r30, 0x03		;check if digit_3
	breq digit_3		;output value to digit_3
digit_0: ;
	ldd r31, Y+0		;store led_display[0] in r18
	out VPORTD_OUT, r31			;output r31 to display
	cbi VPORTA_OUT, 4		;turn ON digit 0
	rjmp end 
digit_1: 
	ldd r31, Y+1		;store led_display[1] in r18
	out VPORTD_OUT, r31			;output r31 to display
	cbi VPORTA_OUT, 5		;turn ON digit 1
	rjmp end 
digit_2: 
	ldd r31, Y+2		;store led_display[2] in r18
	out VPORTD_OUT, r31		;output r31 to display
	cbi VPORTA_OUT, 6		;turn ON digit 2
	rjmp end ; 
digit_3: 
	ldd r31, Y+3		;store led_display[3] in r18
	out VPORTD_OUT, r31		;output r31 to display
	cbi VPORTA_OUT, 7		;turn ON digit 3
end: 
	inc r30 ;		increment r30 - stores digit_num
	sts digit_num, r30		;store r30 in location pointed to be X
	ret 

;***************************************************************************
;* 
;* "hex_to_7seg" - Hexadecimal to Seven Segment Conversion
;*
;* Description: Converts a right justified hexadecimal digit to the seven
;* segment pattern required to display it. Pattern is right justified a
;* through g. Pattern uses 0s to turn segments on ON.
;*
;* Author:			Ken Short
;* Version:			0.1						
;* Last updated:		100322
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:
;* Low registers modified:
;* High registers modified:
;*
;* Parameters: r18: hex digit to be converted
;* Returns: r18: seven segment pattern. 0 turns segment ON
;*
;* Notes: 
;*
;***************************************************************************

hex_to_7seg:
	push ZH
	push ZL
    
	ldi ZH, HIGH(hextable * 2)  ;set Z to point to start of table
    ldi ZL, LOW(hextable * 2)
    ldi r16, $00                ;add offset to Z pointer
	andi r18, 0x0F				;mask for low nibble
    add ZL, r18
    adc ZH, r16
    lpm r18, Z                  ;load byte from table pointed to by Z
	
	pop ZL
	pop ZH
	
	ret

    ;Table of segment values to display digits 0 - F
    ;!!! seven values must be added
hextable: .db $01, $4F, $12, $06, $4C, $24, $20, $0F, $00, $04, $08, $60, $31, $42, $30, $38

;***************************************************************************
;* 
;* "write_0_to_DHT11" - Write 0 to DHT11
;*
;* Description:
;* This subroutine writes a 0 to the DATA line PB0 and leaves the DATA line
;* at the logic 0 level
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:		2
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;* Leaves a logic 0 level at the DATA line
;* 
;* Notes: 
;* Leaves PB0 configured as an output
;*
;***************************************************************************

write_0_to_DHT11:
	sbi VPORTB_DIR, 0	;change PB0 to an output pin
	cbi VPORTB_OUT, 0	;output a 0 to dataline
	ret

;***************************************************************************
;* 
;* "write_1_to_DHT11" - Write 1 to DHT11
;*
;* Description:
;* This subroutine causes the DATA line PB0 to be pulled up to a 1 by an 
;* external pull-up resistor. Can be used to make PB0 function like an open
;* drain pin
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:		1
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;* Leaves the the DATA line at a floating value to be pulled up to a 1 by
;* an external pull-up resistor 
;* 
;* Notes: 
;* Leaves PB0 configured as an input
;*
;***************************************************************************

write_1_to_DHT11:
	cbi VPORTB_DIR, 0	;change PB0 to input so it is high-impedence - DHT11 pulls dataline to 1
	ret

;***************************************************************************
;* 
;* "var_us_delay" - Variable Microsecond Delay 
;*
;* Description:
;* This subroutine returns an r16 * 1us delay for AVR128DB48 @ 4.00 MHz
;* Return
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:	4 * r16		
;* Low registers modified:	
;* High registers modified:		r16
;*
;* Parameters:	r16
;* 
;* Returns:
;* An r16 * 1us delay 
;* 
;* Notes: 
;* r16 contains 0x00 after calling this subroutine
;*
;***************************************************************************

var_us_delay:
	nop
	loop:
		nop
		dec r16
		brne loop
		ret

;***************************************************************************
;* 
;* "var_ms_delay" - Variable Millisecond Delay 
;*
;* Description:
;* This subroutine returns an r16 * 0.1ms delay for AVR128DB48 @ 4.00 MHz
;* Return
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:	400 * r16		
;* Low registers modified:	
;* High registers modified:		r16
;*
;* Parameters:	r16
;* 
;* Returns:
;* An r16 * 0.1ms delay 
;* 
;* Notes: 
;* r16 contains 0x00 after calling this subroutine
;*
;***************************************************************************

var_ms_delay:
	nop
	outer_loop:
		ldi r17, 132
	inner_loop:
		nop
		dec r17
		brne inner_loop
		dec r16
		brne outer_loop
		ret

;***************************************************************************
;* 
;* "delay_50us" - Delay 50 microseconds
;*
;* Description:
;* This subroutine returns a 50 microsecond delay for AVR128DB48 @ 4.00 MHz
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:		200
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;* Returns a 50.00us delay for AVR128DB48 @ 4.00 MHz
;* 
;* Notes: 
;*
;***************************************************************************

delay_50us:		
	push r16		;1 clk - save contents of r16
	ldi r16, 49		;1 clk - load r16 with loop control variable
	rcall var_us_delay
	pop r16		;2 clk - restore r16
	ret

;***************************************************************************
;* 
;* "delay_18ms" - Delay 18 milliseconds
;*
;* Description:
;* This subroutine returns an 18.001 millisecond delay for AVR128DB48 @ 4.00 MHz
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:		72004
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;* Returns a 18.001 millisecond delay for AVR128DB48 @ 4.00 MHz
;* 
;* Notes: 
;*
;***************************************************************************

delay_18ms:		
	push r16	;1 clk - save contents of r16
	ldi r16, 180	;1 clk - load r16 with loop control variable
	rcall var_ms_delay
	pop r16		;2 clks - restore contents of r16
	ret

;***************************************************************************
;* 
;* "delay_20us" - Delay 20 microseconds
;*
;* Description:
;* This subroutine returns a 20 microsecond delay for AVR128DB48 @ 4.00 MHz
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:		80
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;* Returns a 20.00us delay for AVR128DB48 @ 4.00 MHz
;* 
;* Notes: 
;*
;***************************************************************************

delay_20us:	
	push r16		;1 clk - save contents of r16
	ldi r16, 19		;1 clk - load r16 with loop control variable
	rcall var_us_delay
	pop r16		;2 clk - restore r16
	ret

;***************************************************************************
;* 
;* "send_start" - Send Start
;*
;* Description:
;* Signals to the DHT11 to send back its five bytes of data. Sends a 0 to the 
;* DATA line for 18ms and then release the DATA line. This negative pulse is the 
;* start signal for the DHT11 to send data. The end of the start signal is a 1 from
;* the microcontroller for 20us.
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:		723047	
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;*  
;* Notes: 
;* 
;***************************************************************************

send_start:
	cli		;disable global interrupts

	rcall write_0_to_DHT11		;send 0 to DATA line
	rcall delay_18ms	;delay 18ms
	rcall write_1_to_DHT11		;release DATA line
	rcall delay_20us	;signal end of start signal

	sei		;enable global interrupts

	ret

;***************************************************************************
;* 
;* "wait_for_response" - Wait for Response 
;*
;* Description:
;* Waits for a response signal from the DHT11 so that data bits can be decoded. 
;* The response from the DHT11  consists of the DHT11 driving the DATA line low 
;* for 80us and then high for 80us.
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:
;* High registers modified:
;*
;* Parameters:
;*
;* Returns:
;*  
;* Notes: 
;* 
;***************************************************************************

wait_for_response:
	cli		;disable globale interrupts

	push r16	;save r16 contents
;wait for DHT11 to drive DATA line low
	wait_for_response_low:
		rcall read_DHT11
		sbrc r16, 0
		rjmp wait_for_response_low
;delay for 80us
	rcall delay_20us
	rcall delay_20us
	rcall delay_20us
;check if DATA line is stable after 60us
	check_if_still_low:
		rcall read_DHT11
		sbrc r16, 0
		rjmp wait_for_response	;DATA line was low for less than 80us - invalid response
	rcall delay_20us
;wait for DHT11 to drive DATA line high
	wait_for_response_high:
		rcall read_DHT11
		sbrs r16, 0
		rjmp wait_for_response_high
;delay for 80us
	rcall delay_20us
	rcall delay_20us
	rcall delay_20us
;check if DATA line is stable after 60us
	check_if_still_high:
		rcall read_DHT11
		sbrs r16, 0
		rjmp wait_for_response_high
;check if response from DHT11 is completeafter 75us
	ldi r16, 15
	rcall var_us_delay
	rcall read_DHT11
	sbrs r16, 0
	rjmp wait_for_response_low
;response is sucessful
	pop r16		;restore contents of r16

	sei		;enable global interrupts
			
	ret

;***************************************************************************
;* 
;* "read_DHT11" - Read DHT11 
;*
;* Description:
;* Returns the logical value of the DATA line in bit 0 of r16
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified: 
;* High registers modified:		r16
;*
;* Parameters:	
;* VPORTB_IN register
;*
;* Returns:
;* Returns the logical value of the DATA line in bit 0 of r16
;* 
;* Notes: 
;* 
;***************************************************************************

read_DHT11:
	in r16, VPORTB_IN	;load r16 with PORTB values
	andi r16, 0x01		;mask for PB0 and bit 0
	ret

;***************************************************************************
;* 
;* "read_DHT11_data_bit" - Read DHT11 data bit 
;*
;* Description:
;* Decodes a data bit from the DHT11 and determines if it is a 1 or 0. The data 
;* bit is returned as a byte value in r16 that is 0x00 for a 0 data bit and is 0x01 
;* for a 1 data bit.
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified:		r16
;*
;* Parameters:	
;* VPORTB_IN register
;*
;* Returns:
;* The data bit as a byte value in r16 that is 0x00 for a 0 data bit and is 0x01 
;* for a 1 data bit.
;* 
;* Notes: 
;* 
;***************************************************************************

read_DHT11_data_bit:
;wait for start of data bit
	wait_for_1:
		rcall read_DHT11
		sbrs r16, 0
		rjmp wait_for_1
;delay 14us
	ldi r16, 14
	rcall var_us_delay
;check if still 1
	rcall read_DHT11
	sbrs r16, 0	
	rjmp wait_for_1		;go back to beginning if 0, pulse width <= 14us. undefined value
;delay 10us : 24us total
	ldi r16, 10
	rcall var_us_delay
;check if still 1
	rcall read_DHT11
	sbrs r16, 0	
	rjmp wait_for_1		;go back to beginning if 0, pulse width <= 24us. undefined value	
;delay 6us : 30us total
	ldi r16, 6
	rcall var_us_delay
;check if still 1
	rcall read_DHT11
	sbrs r16, 0	
	rjmp bit_is_0		;pulse width is between 24 and 30 microseconds. Consider bit as 0 
;delay 30us : 60us total
	ldi r16, 30
	rcall var_us_delay
;check if still 1
	rcall read_DHT11
	sbrs r16, 0	
	rjmp wait_for_1		;go back to beginning if 0, pulse width between 30-60 microseconds. undefined value
;delay 6us : 66us total
	ldi r16, 6
	rcall var_us_delay
;check if still 1
	rcall read_DHT11
	sbrs r16, 0	
	rjmp wait_for_1		;go back to beginning if 0, pulse width 60-66. undefined value
;delay 6us : 72us total
	ldi r16, 6
	rcall var_us_delay
;check if still 1
	rcall read_DHT11
	sbrc r16, 0	
	rjmp wait_for_1		;go back to beginning if 1, pulse width > 72us. undefined value
	rjmp bit_is_1		;pulse width is between 66-72. Consider bit as 1 	
		
bit_is_0:
	ldi r16, 0x00	;DHT11 sent a 0, pulse width 24-30us
	ret
bit_is_1:
	ldi r16, 0x01	;DHT11 sent a 1, pusle width 66-72us
	ret

;***************************************************************************
;* 
;* "get_byte" - Get Byte
;*
;* Description:
;* Reads a sequence of eight data bits from the DHT11 and stores it in r18
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified: r18
;*
;* Parameters:	
;* 
;*
;* Returns:
;* A byte from the DHT11 stored in r18 
;* 
;* Notes: 
;* The first bits read are stored in the most significant bits of r18 and the last
;* bits read are stored in the least significant bits of r18
;*
;***************************************************************************

get_byte:
	push r16	;store registers
	push r17
	ldi r18, 0x00	;initialize r18
	ldi r17, 8		;loop control variable
	get_byte_loop:
		rcall read_DHT11_data_bit	;read data bit from DHT11
		ror r16		;right shift data bit into carry 
		rol r18		;left shift carry into register
		rcall delay_50us	;wait 50us before reading next bit
		dec r17		;decrement loop control variable
		brne get_byte_loop
	pop r17		;restore registers
	pop r16
	ret

;***************************************************************************
;* 
;* "get_measured_data" - Get Measured Data
;*
;* Description:
;* Perform five byte reads and puts five bytes of data from the DHT11 into the 
;* 5 byte array measured_data 
;*
;* Author:			Omar Mohamed
;* Version:			0.1
;* Last updated:		11-13-2022
;* Target:			AVR128DB48
;* Number of words:
;* Number of cycles:			
;* Low registers modified:	
;* High registers modified: 
;*
;* Parameters:	
;* 
;* Returns:
;* 5 bytes of data from the DHT11 stored in memory in the array measured_data
;* 
;* Notes: 
;* 
;***************************************************************************

get_measured_data:
	cli		;clear global interrupt enable

	push ZL		;store registers
	push ZH
	push r16
	push r18
	ldi r16, 5		;loop control variable
	ldi ZL, LOW (measured_data)		;initialize Z-pointer to measured_data[0], i = 0
	ldi ZH, HIGH (measured_data)
	store_data:
		rcall get_byte		;store byte from DHT11 in r18
		st Z+, r18		;store byte in array[i] - post increment i
		dec r16		;decrement loop control variable
		brne store_data		
	pop r18		;restore registers
	pop r16
	pop ZH
	pop ZL
	
	sei		;set global interrupt enable
	
	ret