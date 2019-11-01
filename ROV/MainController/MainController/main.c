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

struct PIDdata {
	float uLast;
	float errLast1, errLast2;
};

float pid(struct PIDdata *plant, float err, float p, float i, float d) {
	float u = plant->uLast + p*(1+i+d) * err - p*(1+2*d) * plant->errLast1 + p*d*plant->errLast2;
	plant->errLast2 = plant->errLast1;
	plant->errLast1 = err;
	plant->uLast = u;
	return u;
}

int main() {
	//Init I/O
	DDRD = 0xFF;
	DDRC = 0x00;
	DDRB = 0xFF;
	
	//Init pref
	initUART();
	initADC();
	initPPM();

	//Init communication system
	uint8_t txPacket[32], rxPacket[32];
	txPacket[31] = pgm_read_word(&( SOFTWARE_INFO[0] ));

	txPacket[0] =  'A';
	txPacket[1] =  'D';
	txPacket[2] =  'C';
	txPacket[3] =  ' ';
	txPacket[4] =  'V';
	txPacket[5] =  'o';
	txPacket[6] =  'l';
	txPacket[7] =  't';
	txPacket[9] =  'a';
	txPacket[9] =  'g';
	txPacket[10] = 'e';
	txPacket[11] = ' ';
	txPacket[12] = 'r';
	txPacket[13] = 'e';
	txPacket[14] = 'a';
	txPacket[15] = 'd';
	txPacket[16] = 'i';
	txPacket[17] = 'n';
	txPacket[18] = 'g';
	txPacket[19] = ':';
	txPacket[20] = ' ';
	txPacket[21] = '9';
	txPacket[22] = '.';
	txPacket[23] = '9';
	txPacket[24] = '9';
	txPacket[25] = 'V';
	txPacket[26] = ' ';
	txPacket[27] = ' ';
	txPacket[28] = ' ';
	txPacket[29] = ' ';
	txPacket[30] = ' ';
	txPacket[31] = ' ';
	initSysTimer();

	sei();
	for(;;) {
		//Get ADC
		uint16_t adcValue = getADC(0);
		float adcVoltage = adcValue * 4.96 /1024.0;

		uint16_t adcVoltageInt = (uint16_t)( adcVoltage * 100 ); //Range 0 - 500
		uint8_t v1Char = adcVoltageInt % 10;
		uint8_t v10Char = adcVoltageInt / 10 % 10;
		uint8_t v100Char = adcVoltageInt / 100;
		
		txPacket[21] = v100Char + '0';
		txPacket[23] = v10Char + '0';
		txPacket[24] = v1Char + '0';
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