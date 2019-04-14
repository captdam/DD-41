;App RAM - LCD display buffer
	LCD_BUFFER	XDATA	0x00			;LCD display buffer (4 * 16)

;App RAM - Tx data
	TX_BUFFER	XDATA	0xE0			;UART Tx buffer (15 data + 1 checksum)
	
	FB_VALVE	DATA	0x30
	DIR_VALVE	DATA	0x31
	FUNC		DATA	0x32
	C_OUT		DATA	0x33
	ENGINE_POWER	DATA	0x34
	C_PWM		DATA	0x35
	PRESSURE_DEST	DATA	0x36
	PITCH_DEST	DATA	0x38
	COMPASS_DEST	DATA	0x3A
	TX_REV_0	DATA	0x3C
	TX_REV_1	DATA	0x3D
	TX_REV_2	DATA	0x3E
	TX_CHECKSUM	DATA	0x3F	;Not used

;App RAM - Rx data
	RX_BUFFER	XDATA	0xF0			;UART Rx buffer (15 data + 1 checksum)
	
	PRESSURE_REAL	DATA	0x40
	PITCH_REAL	DATA	0x42
	COMPASS_REAL	DATA	0x44
	TEMPERATURE	DATA	0x46
	BAT_VOLTAGE	DATA	0x48
	C_IN		DATA	0x4A
	RX_REV_0	DATA	0x4B
	RX_REV_1	DATA	0x4C
	RX_REV_2	DATA	0x4D
	RX_REV_3	DATA	0x4E
	RX_CHECKSUM	DATA	0x4F	;Not used

;App RAM - Local variables
	BIT_WORKSPACE	DATA	0x2F
	DIGI_BUFFER_L	DATA	0x50
	DIGI_BUFFER_H	DATA	0x51
	DIGI_BUFFER_E	DATA	0x52
	
	SCAN_DIVIDER	DATA	0x53
	
;	FP		DATA	0x7F			;Frame pointer


	
	