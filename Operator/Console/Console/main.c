#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <util/delay_basic.h>

volatile const char SOFTWARE_INFO[] PROGMEM = "DD-41 project. Operator-side console. Author: Captdam. 2019.";


/************************************************************************/
/* Config                                                               */
/************************************************************************/

#define CPU_FREQ	16000000


/************************************************************************/
/* Application SFRs                                                     */
/************************************************************************/


/************************************************************************/
/* ROM consts, lookup tables                                            */
/************************************************************************/


/************************************************************************/
/* Code segment                                                         */
/************************************************************************/

#include "../../../Communication/Software/communication.c"
#include "adc.c"
#include "spi.c"
#include "osd.c"


// UART ------------------------------------------------------------------

//Send a data, wait if buffer not empty
void sendSerialSync(uint8_t data) {
	while ( !(UCSR0A & (1<<UDRE0)) ); //Wait until last word send
	UDR0 = data;
}

//Request data, wait until data arrives
uint8_t requestSerialSync() {
	while ( !(UCSR0A & (1<<RXC0)) ); //Wait until data received
	return UDR0;
}

// Data format -----------------------------------------------------------

//BCD to binary
uint8_t d2b(uint8_t decimal) {
	return (((decimal & 0xF0) >> 4 ) * 10 ) + (decimal & 0x0F); //Normalizes 10s * 10 + 1s
}

//Binary to BCD
uint8_t b2d(uint8_t binary) {
	return ( (binary / 10) << 4 ) | binary % 10; //10s in higher nipple, 1s in lower nipple
}

//Binary to hex string
uint16_t b2h(uint8_t binary) {
	uint8_t high = binary >> 4;
	uint8_t low = binary & 0x0F;
	if (high > 9)
		high = high -10 + 'A';
	else
		high = high + '0';
	if (low > 9)
		low = low -10 + 'A';
	else
		low = low + '0';
	return (high<<8) | low;
}


// MAIN ROUTINES ---------------------------------------------------------

//Init hardwares
int main() {
	//Init I/O
	DDRD = 0x00; //Digital in
	DDRC = 0x00; //ADC
	DDRB = 0xFF; //SPI CS, System status output

	PORTB |= (1<<2) | (1<<1); //CS = 1, non-selected
	
	//Init pref
	initUART();
	initADC();
	initSPIMaster();
	for (uint8_t i = 0; i < 25; i++) _delay_loop_2(65535); //Waiting for external device booting

	//Init communication system
	uint8_t txPacket[32], rxPacket[32];
	txPacket[31] = pgm_read_word(&( SOFTWARE_INFO[0] ));
	placePacket(txPacket);
//	initSysTimer();

//	sei();

	initOSD();
	
	uint8_t line0[] = "OSD TEST";
	for (uint8_t i = 0; i < sizeof(line0) - 1; i++) line0[i] = charEncodeOSD(line0[i]);
	writeSringOSD(0, 5, line0, sizeof(line0) - 1);
	
	uint8_t line1[] = "ROV B. V. 12.7V";
	for (uint8_t i = 0; i < sizeof(line1) - 1; i++) line1[i] = charEncodeOSD(line1[i]);
	writeSringOSD(1, 0, line1, sizeof(line1) - 1);
	
	uint8_t line2[] = "Ctl B. V. 14.2V";
	for (uint8_t i = 0; i < sizeof(line2) - 1; i++) line2[i] = charEncodeOSD(line2[i]);
	writeSringOSD(2, 0, line2, sizeof(line2) - 1);
	
	uint8_t line3[] = "P  13.2  R -27.5";
	for (uint8_t i = 0; i < sizeof(line3) - 1; i++) line3[i] = charEncodeOSD(line3[i]);
	writeSringOSD(3, 0, line3, sizeof(line3) - 1);
	
	uint8_t line4[] = "A/P activated";
	for (uint8_t i = 0; i < sizeof(line4) - 1; i++) line4[i] = charEncodeOSD(line4[i]);
	writeSringBlinkOSD(11, 0, line4, sizeof(line4) - 1);
	
/*	for (;;) {
		for (uint8_t i = 0; i < 25; i++) _delay_loop_2(65535);
		turnOnOSD();
		for (uint8_t i = 0; i < 25; i++) _delay_loop_2(65535);
		turnOffOSD();
	}*/


	for (;;) {
		uint16_t j[4];
		j[0] = getADC(0);
		j[1] = getADC(1);
		j[2] = getADC(2);
		j[3] = getADC(3);

		uint8_t display[] = "xxxx xxxx xxxx xxxx";


		for (uint8_t i = 0; i < 4; i++) {
			display[ i*5 + 0 ] = j[i] / 1000 + '0';
			display[ i*5 + 1 ] = j[i] / 100 % 10 + '0';
			display[ i*5 + 2 ] = j[i] / 10 % 10 + '0';
			display[ i*5 + 3 ] = j[i] % 10 + '0';
		}

		for (uint8_t i = 0; i < sizeof(display) - 1; i++) display[i] = charEncodeOSD(display[i]);
		writeSringOSD(8, 0, display, sizeof(display) - 1);

		PINB = (1<<0);
	}
	
	return 0;
}


/************************************************************************/
/* Known issues                                                         */
/************************************************************************/