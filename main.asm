/*
 * rgb_dmx.asm
 *
 *   8 channel dmx receiver with 10-bit resolution bit-angle-modulation (BAM)
 *
 */ 


.include "m32def.inc"

.equ F_CPU = 16000000							; cpu speed

.def tmp = r16
.def flags = r17
.def timerCompareCount = r18
.def OVFCompL = r19
.def OVFCompH = r20
.def SREGBuffer = r21
.def dmxByte = r22

.def tmpPortByte0 = r2
.def tmpPortByte1 = r3
.def tmpPortByte2 = r4
.def tmpPortByte3 = r5
.def tmpPortByte4 = r6
.def tmpPortByte5 = r7
.def tmpPortByte6 = r8
.def tmpPortByte7 = r9
.def tmpPortByte8 = r10
.def tmpPortByte9 = r11

.def tmpLUTByte0 = r12
.def tmpLUTByte1 = r13

.def dmxAddress0 = r23
.def dmxAddress1 = r24

.equ updatePortFlag = 0
.equ updatePortValuesFlag = 1
.equ dmxStartByteFlag = 2
.equ dmxStartAddrFlag = 3
.equ dmxValueFlag = 4

.dseg
DMXValues:		.byte 8							; stores raw dmx values
PortValues:		.byte 10						; byte n: all nth-bits from 10-bit intesity values

.cseg
.org 0x00										; entrypoint
	rjmp	main

.org OC1Aaddr									; timer 0 overflow
    rjmp	Timer1Compare

.org URXCaddr									; receive interrupt
    rjmp	DMXReceive

; ----------------------------------------------------------------------------------------

main:
	ldi		tmp, LOW(RAMEND)					; init stack
	out		SPL, tmp
	ldi		tmp, HIGH(RAMEND)
	out		SPH, tmp

	rcall	defaultValues

	rcall	initPorts
	rcall	initTimer
	rcall	initUART

	ldi		OVFCompH, 0x00				        ; preset timer compare to 1
	ldi		OVFCompL, 0x01
	out		OCR1AH, OVFCompH
	out		OCR1AL, OVFCompL

	ldi		timerCompareCount, 1				; force update in next cycle
	sbr		flags, (1 << updatePortFlag)		; set update flag

	rcall	updatePortValues

	sei											; enable interrupts

loop:											; main loop
	sbrc	flags, updatePortFlag
	rcall	updatePort
	sbrc	flags, updatePortValuesFlag
	rcall	updatePortValues
	rjmp	loop

; ----------------------------------------------------------------------------------------

initPorts:
	ldi		tmp, 0b11111111						; PORTA output
	out		DDRA, tmp
	
	ldi		tmp, 0b00000000						; default off
	out		PORTA, tmp

	ldi		tmp, 0b00000100						; max 485 read enable
	out		DDRD, tmp
	
	ldi		tmp, 0b10000000						; dip pull up
	out		PORTD, tmp

	ldi		tmp, 0b11111111						; dip pull up
	out		PORTC, tmp

	ret

; ----------------------------------------------------------------------------------------

initTimer:
	ldi     tmp, (1 << CS11) | (1 << CS10) | (1 << CTC1)
    out     TCCR1B, tmp							; timer 1 prescaler 64 + clear timer on compare

	ldi     tmp, 1 << OCIE1A 					; enable output compare interrupt for timer 1
    out     TIMSK, tmp

	ret

; ----------------------------------------------------------------------------------------

initUART:
	ldi 	tmp, ((F_CPU/4000000)-1)			; 250 kbaud
    out 	UBRRL, tmp
    clr 	tmp
    out 	UBRRH, tmp

    ldi 	tmp, (1 << URSEL) | (3 << UCSZ0) | (1 << USBS)
    out 	UCSRC, tmp
	in		tmp, UDR
 	clr		tmp
	out		UCSRA, tmp
	sbi 	UCSRB, RXCIE
	sbi 	UCSRB, RXEN
	
	ret

; ----------------------------------------------------------------------------------------

DMXReceive:
	in		SREGBuffer, SREG					; save SREG
	push	tmp
	
	in		tmp, UCSRA							; get uart status register
	in		dmxByte, UDR						; get byte

	sbrc	tmp, FE								; check for frame error
	rjmp	DMXReceive_frameError
	
												; state machine
	sbrc	flags, dmxValueFlag
	rjmp	DMXReceive_value

	sbrc	flags, dmxStartAddrFlag
	rjmp	DMXReceive_startAddr

	sbrc	flags, dmxStartByteFlag
	rjmp	DMXReceive_startByte

DMXReceive_end:
	pop		tmp
	out		SREG, SREGBuffer					; restore SREG
	reti

DMXReceive_startByte:
	tst		dmxByte								; if startbyte = 0x00
	brne	DMXReceive_reset
	
	in		dmxAddress0, PINC					; PINC address bits 1-8 (switch 2-9)
	com		dmxAddress0							; active low (sets carry 1)
	sbic	PIND, 7								; if switch 1 is off
	clc											; -> clear carry
	rol		dmxAddress0							; rotate left
	rol		dmxAddress1

	sbr		flags, (1 << dmxStartAddrFlag)
	rjmp	DMXReceive_end


DMXReceive_startAddr:
	subi	dmxAddress0, 1						; decrement start address
	sbci	dmxAddress1, 0
	brne	DMXReceive_end						; check if we are at our start address
	
	ldi		YL, LOW(DMXValues)					; init pointer
	ldi		YH, HIGH(DMXValues)
	ldi		dmxAddress0, 8						; 8 channels to receive
	
	sbr		flags, (1 << dmxValueFlag)

DMXReceive_value:
	st		Y+, dmxByte							; store value
	dec		dmxAddress0							; decrement counter
	brne	DMXReceive_end						; last value?

	rjmp	DMXReceive_reset

DMXReceive_frameError:
	sbr		flags, (1 << dmxStartByteFlag)		; break detected
	cbi		UCSRA, FE							; clear frame error bit

	rjmp	DMXReceive_end

DMXReceive_reset:
	cbr		flags, (1 << dmxStartByteFlag)		; reset flags, wait for break
	cbr		flags, (1 << dmxStartAddrFlag)
	cbr		flags, (1 << dmxValueFlag)
	rjmp	DMXReceive_end


; ----------------------------------------------------------------------------------------

Timer1Compare:
	in		SREGBuffer, SREG					; save SREG

	sbr		flags, (1 << updatePortFlag)		; set update flag

	out		SREG, SREGBuffer					; restore SREG
	reti

; ----------------------------------------------------------------------------------------

resetTimer:
	ldi		OVFCompH, 0x00				        ; preset timer compare to 1
	ldi		OVFCompL, 0x01
	ldi		timerCompareCount, 10				; 10 bits
	out		OCR1AH, OVFCompH
	out		OCR1AL, OVFCompL

	ret

; ----------------------------------------------------------------------------------------

updatePort:
	cli											; disable interrupts
	cbr		flags, (1 << updatePortFlag)		; clear update flag
	
	dec		timerCompareCount					; check
	breq	updatePort_reset

	cpi		timerCompareCount, 1
	brne	updatePort_nextCycle
	sbr		flags, (1 << updatePortValuesFlag)	; set update values flag when we are in the longest cycle (1024*64 clocks)
	
updatePort_nextCycle:
	lsl		OVFCompL							; shift timer compare left
	rol		OVFCompH

	out		OCR1AH, OVFCompH					; set timer compare
	out		OCR1AL, OVFCompL

	rjmp	updatePort_update

updatePort_reset:
	ldi		OVFCompH, 0x00				        ; preset timer compare to 1
	ldi		OVFCompL, 0x01
	ldi		timerCompareCount, 10				; 10 bits
	out		OCR1AH, OVFCompH
	out		OCR1AL, OVFCompL

updatePort_update:
	ld		tmp, X+								; push out bits
	out		PORTA, tmp

	sei											; re-enable interrupts
	ret

; ----------------------------------------------------------------------------------------

defaultValues:
	ldi		ZL, LOW(DMXValues)					; dmx values after init
	ldi		ZH, HIGH(DMXValues)
	
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z+, tmp
	ldi		tmp, 0
	st		Z, tmp

	ret

; ----------------------------------------------------------------------------------------

updatePortValues:								; convert dmx values to bam format
	cli

	cbr		flags, (1 << updatePortValuesFlag)	; clear update port values flag

	ldi		XL, LOW(DMXValues)
	ldi		XH, HIGH(DMXValues)

												; dmx byte 0
	ld		tmp, X+								; convert 8 bit dmx values to 10 bit bam
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1						; load 10th bit from lut result
	bld		tmpPortByte9, 0						; put into 10th port byte bit 0
	bst		tmpLUTByte1, 0						; load 9th bit from lut result
	bld		tmpPortByte8, 0						; put into 9th port byte bit 0
	bst		tmpLUTByte0, 7						; ...
	bld		tmpPortByte7, 0
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 0
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 0
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 0
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 0
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 0
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 0
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 0
	
												; dmx byte 1
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 1						; put into 10th port byte bit 1
	bst		tmpLUTByte1, 0						; ...
	bld		tmpPortByte8, 1
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 1
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 1
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 1
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 1
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 1
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 1
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 1
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 1
	
												; dmx byte 2
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 2
	bst		tmpLUTByte1, 0
	bld		tmpPortByte8, 2
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 2
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 2
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 2
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 2
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 2
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 2
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 2
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 2
	
												; dmx byte 3
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 3
	bst		tmpLUTByte1, 0
	bld		tmpPortByte8, 3
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 3
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 3
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 3
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 3
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 3
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 3
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 3
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 3
	
												; dmx byte 4
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 4
	bst		tmpLUTByte1, 0
	bld		tmpPortByte8, 4
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 4
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 4
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 4
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 4
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 4
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 4
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 4
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 4
	
												; dmx byte 5
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 5
	bst		tmpLUTByte1, 0
	bld		tmpPortByte8, 5
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 5
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 5
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 5
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 5
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 5
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 5
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 5
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 5
	
												; dmx byte 6
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 6
	bst		tmpLUTByte1, 0
	bld		tmpPortByte8, 6
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 6
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 6
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 6
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 6
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 6
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 6
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 6
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 6
	
												; dmx byte 7
	ld		tmp, X+
	rcall	DMXLUTWord

	bst		tmpLUTByte1, 1
	bld		tmpPortByte9, 7
	bst		tmpLUTByte1, 0
	bld		tmpPortByte8, 7
	bst		tmpLUTByte0, 7
	bld		tmpPortByte7, 7
	bst		tmpLUTByte0, 6
	bld		tmpPortByte6, 7
	bst		tmpLUTByte0, 5
	bld		tmpPortByte5, 7
	bst		tmpLUTByte0, 4
	bld		tmpPortByte4, 7
	bst		tmpLUTByte0, 3
	bld		tmpPortByte3, 7
	bst		tmpLUTByte0, 2
	bld		tmpPortByte2, 7
	bst		tmpLUTByte0, 1
	bld		tmpPortByte1, 7
	bst		tmpLUTByte0, 0
	bld		tmpPortByte0, 7

	ldi		XL, LOW(PortValues)					; pointer to port values
	ldi		XH, HIGH(PortValues)
	
	st		X+, tmpPortByte0					; store new port values in memory
	st		X+, tmpPortByte1
	st		X+, tmpPortByte2
	st		X+, tmpPortByte3
	st		X+, tmpPortByte4
	st		X+, tmpPortByte5
	st		X+, tmpPortByte6
	st		X+, tmpPortByte7
	st		X+, tmpPortByte8
	st		X,  tmpPortByte9

	ldi		XL, LOW(PortValues)					; reset pointer for next cycle
	ldi		XH, HIGH(PortValues)
	
	sei
	ret

; ----------------------------------------------------------------------------------------

DMXLUTWord:
	ldi		ZL, LOW(dimLUT << 1)				; init pointer for lut
	ldi		ZH, HIGH(dimLUT << 1)
	
	clr		r0									; offset * 2 -> words
	lsl		tmp
	rol		r0
	add		ZL, tmp								; calculate offset
	adc		ZH, r0
	lpm		tmpLUTByte0, Z+						; lsb is first in memory
	lpm		tmpLUTByte1, Z

	ret

dimLUT:	.dw   0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2, \
			  2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2,   2, \
			  2,   2,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   3,   4, \
			  4,   4,   4,   4,   4,   4,   4,   4,   5,   5,   5,   5,   5,   5,   5,   6, \
			  6,   6,   6,   6,   6,   7,   7,   7,   7,   7,   7,   8,   8,   8,   8,   9, \
			  9,   9,   9,  10,  10,  10,  10,  11,  11,  11,  12,  12,  12,  13,  13,  13, \
			 14,  14,  14,  15,  15,  16,  16,  16,  17,  17,  18,  18,  19,  19,  20,  20, \
			 21,  22,  22,  23,  23,  24,  25,  25,  26,  27,  28,  28,  29,  30,  31,  32, \
			 32,  33,  34,  35,  36,  37,  38,  39,  40,  41,  43,  44,  45,  46,  47,  49, \
			 50,  51,  53,  54,  56,  57,  59,  61,  62,  64,  66,  68,  69,  71,  73,  75, \
			 77,  79,  82,  84,  86,  89,  91,  94,  96,  99, 102, 104, 107, 110, 113, 116, \
			120, 123, 126, 130, 133, 137, 141, 145, 149, 153, 157, 161, 166, 170, 175, 180, \
			185, 190, 195, 200, 206, 211, 217, 223, 229, 236, 242, 249, 256, 263, 270, 278, \
			285, 293, 301, 309, 318, 327, 336, 345, 354, 364, 374, 385, 395, 406, 417, 429, \
			441, 453, 465, 478, 491, 505, 519, 533, 548, 563, 578, 594, 610, 627, 644, 662, \
			680, 699, 719, 738, 759, 780, 801, 823, 846, 869, 893, 918, 943, 969, 996, 1023