// At7456E OSD util

// Interrupt during SPI data transmission may corrupt the SPI data, for example:
//     Currently send data to device A. Interrupt, pull CS of device B low, send data to device B
//     Because the CS of device A is still low, data received by device A as well.
// I-flag should not be enabled during init
// After init, the user may to set the I-flag. When the OSD util sending data to OSD module, this util will temperately disable I-flag

//#include "spi.c"

#define OSD_CS_PORT PORTB
#define OSD_CS_PING 1

uint8_t accessDataOSD(uint8_t data);
uint8_t accessAddrDataOSD(uint8_t addr, uint8_t data);

// Init OSD module, config
void initOSD() {
	accessAddrDataOSD(0x00, 0x08); //Turn on OSD, PAL
	accessAddrDataOSD(0x02, 0b00011111); //Horizontal offset
	accessAddrDataOSD(0x03, 0b00011101); //Vertical offset
	accessAddrDataOSD(0x01, 0b01001100); //Flash rate
}

// Turn on or off OSD
void turnOnOSD() {
	cli();
	accessAddrDataOSD(0x00, 0x08); //Turn on OSD, PAL
	sei();
}
void turnOffOSD() {
	cli();
	accessAddrDataOSD(0x00, 0x00); //Turn off OSD
	sei();
}

// Write a string on screen
void writeSringOSD(uint8_t row, uint8_t col, uint8_t codedString[]) {
	cli();
	
	/*
	Due to hardware issue?
	Only 28 character are able to be displayed on screen per line, although it should be 30
	*/
	uint16_t position = row * 30 + col + 1;
	accessAddrDataOSD(0x05, position >> 8); //Write position
	accessAddrDataOSD(0x06, position & 0xFF);
	accessAddrDataOSD(0x04, 0x01); //Position Auto-increment

	uint8_t index = 0;
	do {
		accessDataOSD(codedString[index]);
	} while( codedString[index++] != 0xFF ); //Write 0xFF to terminate
	
	sei();
}

// Write a string on screen with blinking
void writeSringBlinkOSD(uint8_t row, uint8_t col, uint8_t codedString[]) {
	cli();
	
	/*
	Due to hardware issue?
	Only 28 character are able to be displayed on screen per line, although it should be 30
	*/
	uint16_t position = row * 30 + col + 1;
	accessAddrDataOSD(0x05, position >> 8); //Write position
	accessAddrDataOSD(0x06, position & 0xFF);
	accessAddrDataOSD(0x04, 0x11); //Position Auto-increment

	uint8_t index = 0;
	do {
		accessDataOSD(codedString[index]);
	} while( codedString[index++] != 0xFF ); //Write 0xFF to terminate
	
	sei();
}

// Clear screen
void clearOSD() {
	cli();
	for(uint16_t i = 0; i < 480; i++) accessDataOSD(0x00); //Space
	accessDataOSD(0xFF);
	sei();
}

// ASCII to OSD encoded
#define OSD_NC 0xFB //OSD no such char
volatile const char ASCII2OSD[] PROGMEM = {
//	NUL	SOH	STX	ETX	EOT	ENQ	ACK	BEL	BS	HT	LF	VT	FF	CR	S0	S1
	0xFF,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,
//	DLE	DC1	DC2	DC3	DC4	NAK	SYN	ETB	CAN	EM	SUB	ESC	FS	GS	RS	US
	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,
//	SP	!	"	#	$	%	&	'	(	)	*	+	,	-	.	/
	0x00,	OSD_NC,	0x48,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	0x46,	0x3F,	0x40,	OSD_NC,	OSD_NC,	0x45,	0x49,	0x41,	0x47,
//	0	1	2	3	4	5	6	7	8	9	:	;	<	=	>	?
	0x0A,	0x01,	0x02,	0x03,	0x04,	0x05,	0x06,	0x07,	0x08,	0x09,	0x44,	0x43,	0x4A,	OSD_NC,	0x4B,	0x42,
//	@	A	B	C	D	E	F	G	H	I	J	K	L	M	N	O
	0x4C,	0x0B,	0x0C,	0x0D,	0x0E,	0x0F,	0x10,	0x11,	0x12,	0x13,	0x14,	0x15,	0x16,	0x17,	0x18,	0x19,
//	P	Q	R	S	T	U	V	W	X	Y	Z	[	\	]	^	_
	0x1A,	0x1B,	0x1C,	0x1D,	0x1E,	0x1F,	0x20,	0x21,	0x22,	0x23,	0x24,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,
//	`	a	b	c	d	e	f	g	h	i	j	k	l	m	n	o
	OSD_NC,	0x25,	0x26,	0x27,	0x28,	0x29,	0x2A,	0x2B,	0x2C,	0x2D,	0x2E,	0x2F,	0x30,	0x31,	0x32,	0x33,
//	p	q	r	s	t	u	v	w	x	y	z	{	|	}	~	DEL
	0x34,	0x35,	0x36,	0x37,	0x38,	0x39,	0x3A,	0x3B,	0x3C,	0x3D,	0x3E,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,	OSD_NC,
};
//Note: 0x00 (\0) is the endmark of string, 0xFF exits burst write mode of OSD
uint8_t charEncodeOSD(uint8_t ascii) {
	return pgm_read_word(&(ASCII2OSD[ascii]));
}
void stringEncodeOSD(uint8_t* ascii) {
	do {
		*ascii = pgm_read_word(&(ASCII2OSD[ *ascii ]));
	} while (*(ascii++) != 0xFF);
}


// SPI util functions
uint8_t accessDataOSD(uint8_t data) {
	OSD_CS_PORT &= ~(1<<OSD_CS_PING);
	uint8_t returnData = shiftSPIMaster(data);
	OSD_CS_PORT |= (1<<OSD_CS_PING);
	return returnData;
}
uint8_t accessAddrDataOSD(uint8_t addr, uint8_t data) {
	OSD_CS_PORT &= ~(1<<OSD_CS_PING);
	shiftSPIMaster(addr);
	uint8_t returnData = shiftSPIMaster(data);
	OSD_CS_PORT |= (1<<OSD_CS_PING);
	return returnData;
}