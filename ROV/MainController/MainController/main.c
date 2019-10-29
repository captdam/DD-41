#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>

volatile const char SOFTWARE_INFO[] PROGMEM = "DD-41 project. ROV-side main controller. Author: Captdam. 2019.";


/************************************************************************/
/* Config                                                               */
/************************************************************************/

#define RC_CLOCK_FREQ 16000000


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
#include "ppm.c"
#include "adc.c"


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

int main() {
	//Init I/O
	DDRD = 0xFF;
	DDRC = 0x00;
	DDRB = 0xFF;
	
	initUART();
	initADC();
	initPPM();

	uint8_t txPacket[32], rxPacket[32];
	txPacket[31] = pgm_read_word(&( SOFTWARE_INFO[0] ));
	
	//Receive cmd from PC
	initSysTimer();
	sei();
	for(;;) {
		//Send data
		uint16_t adcValue = getADC(0);
		uint16_t adcStringHigh = b2h(adcValue>>8);
		uint16_t adcStringLow = b2h(adcValue&0xFF);
		
		txPacket[0] = adcStringHigh >> 8;
		txPacket[1] = adcStringHigh & 0xFF;
		txPacket[2] = adcStringLow >> 8;
		txPacket[3] = adcStringLow & 0xFF;
		txPacket[4] = 0;
		txPacket[5] = rxPacket[0]-'0';
		txPacket[6] = rxPacket[1]-'0';
		txPacket[7] = rxPacket[2]-'0';
		placePacket(txPacket);

		//Get data
		fetchPacket(rxPacket);
		float cmd = (rxPacket[0]-'0') * 100 +  (rxPacket[1]-'0') * 10 +  (rxPacket[2]-'0'); //Range 0 to 999
		cmd = cmd / 500.0 - 1.0;
		
		setPPM0(cmd);
		setPPM1(cmd);
		setPPM2(cmd);
		setPPM3(cmd);
	}
	
	return 0;
}


/************************************************************************/
/* Known issues                                                         */
/************************************************************************/