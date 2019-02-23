; LCD instruction

__M_LCD_INI		MACRO	CMD			;Write instruction to both LCDs
		CLR	LCD_RS
		CLR	LCD_RW
		SETB	LCD_E0
		SETB	LCD_E1
		MOV	LCD_DATA, CMD
		CLR	LCD_E0
		CLR	LCD_E1
	ENDM

__M_LCD_PREPARE		MACRO
	__M_LCD_INI #0x38				;8-bit interface, 2-line, 5*8 font
	__M_WAIT5000
	__M_LCD_INI #0x38
	__M_WAIT100
	__M_LCD_INI #0x08				;Cursor display off
	__M_WAIT100
	__M_LCD_INI #0x01				;Clear display
	__M_WAIT5000
	__M_LCD_INI #0x06				;Cursor auto-inc (left-to-right write)
	__M_WAIT100
	__M_LCD_INI #0x0C				;Turn on display
	__M_WAIT100
	ENDM

;__M_LCD0_SETCURSOR	MACRO	LINE,INDEX		;Set coursor of LCD0
;	MOV	A, #(LINE*0x40+INDEX)
;	CALL	LCD0_SETCURSOR
;	__M_WAIT100
;	ENDM
;
;__M_LCD0_SETDATA	MACRO	CHARCODE		;Set charcode of LCD0
;	MOV	A, CHARCODE
;	CALL	LCD0_SETDATA
;	__M_WAIT100
;	ENDM
;	
;__M_LCD1_SETCURSOR	MACRO	LINE,INDEX		;Set coursor of LCD1
;	MOV	A, #(LINE*0x40+INDEX)
;	CALL	LCD1_SETCURSOR
;	__M_WAIT100
;	ENDM
;
;__M_LCD1_SETDATA	MACRO	CHARCODE		;Set charcode of LCD1
;	MOV	A, CHARCODE
;	CALL	LCD1_SETDATA
;	__M_WAIT100
;	ENDM
;

LCD0_SETCURSOR:
	CLR	LCD_RS
	CLR	LCD_RW
	SETB	LCD_E0
	ORL	A, #0x80				;DB7 = 1, DB6-0 = Address
	MOV	LCD_DATA, A
	CLR	LCD_E0
	RET

LCD1_SETCURSOR:
	CLR	LCD_RS
	CLR	LCD_RW
	SETB	LCD_E1
	ORL	A, #0x80				;DB7 = 1, DB6-0 = Address
	MOV	LCD_DATA, A
	CLR	LCD_E1
	RET

LCD0_SETDATA:
	SETB	LCD_RS
	CLR	LCD_RW
	SETB	LCD_E0
	MOV	LCD_DATA, A
	CLR	LCD_E0
	RET

LCD1_SETDATA:
	SETB	LCD_RS
	CLR	LCD_RW
	SETB	LCD_E1
	MOV	LCD_DATA, A
	CLR	LCD_E1
	RET


__M_LCD_WRITEBUFFER	MACRO	LCD, LINE, INDEX
	MOV	DPTR,	#(LCD*(LCD1_LINE0-LCD0_LINE0)+LINE*(LCD0_LINE1-LCD0_LINE0)+INDEX)
	MOVX	@DPTR, A
	ENDM

__M_LCD_APPENDBUFFER	MACRO
	INC	DPTR
	MOVX	@DPTR, A
	ENDM


__M_HIGH2ASCII		MACRO
	ANL	A, #0xF0
	SWAP	A
	ADD	A, #0x30
	ENDM

__M_LOW2ASCII		MACRO
	ANL	A, #0x0F
	ADD	A, #0x30
	ENDM