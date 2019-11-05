#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>

volatile const char SOFTWARE_INFO[] PROGMEM = "DD-41 project. ROV-side main controller. Author: Captdam. 2019.";


/************************************************************************/
/* Config                                                               */
/************************************************************************/

#define CPU_FREQ 16000000


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
#include "../../../AVR_Common/adc.c"
#include "../../../AVR_Common/twi.c"


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

#define I2C_MPU_ID 0b1101000
#define I2C_MPU_ADDR_WHOAMI 0x75
#define I2C_MPU_CONTENT_WHOAMI 0x68
#define I2C_MPU_ADDR_PWRMAGMT1 0x6B
#define I2C_MPU_ADDR_SIGNALPATHRESET 0x68
#define I2C_MPU_ADDR_SLV0CTR 0x25
#define I2C_MPU_ADDR_DATA 0x3B
#define I2C_MPU_AMOUNT_DATA 20 //9 axis + 1 temperature, 2 bytes each

int main() {
	//Init I/O
	DDRD = 0xFF;
	DDRC = 0x00;
	DDRB = 0xFF;
	
	//Init pref
	initUART();
	initADC();
	initPPM();
	initI2CMaster(40000);

	uint8_t mpuId;
	startI2CMaster();
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_WRITE);
	writeI2CMaster(I2C_MPU_ADDR_WHOAMI);
	startI2CMaster();
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_READ);
	readI2CMaster(&mpuId,I2C_M_RETURN_NAK); //Read into void and return NACK to terminate I2C
	stopI2CMaster();
	sendSerialSync(mpuId);
//	for(;;) {}



	//Reset MPU module
	startI2CMaster(); //Reset MPU
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_WRITE);
	writeI2CMaster(I2C_MPU_ADDR_PWRMAGMT1);
	writeI2CMaster(0x00);
	stopI2CMaster();
	
	//Reset MPU signal path
	startI2CMaster();
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_WRITE);
	writeI2CMaster(I2C_MPU_ADDR_SIGNALPATHRESET);
	writeI2CMaster(0x07);
	stopI2CMaster();

	//Init communication system
	uint8_t txPacket[32], rxPacket[32];
	txPacket[31] = pgm_read_word(&( SOFTWARE_INFO[0] ));

	txPacket[0] =  'P';
	txPacket[1] =  'i';
	txPacket[2] =  't';
	txPacket[3] =  'c';
	txPacket[4] =  'h';
	txPacket[5] =  ':';
	txPacket[6] =  '9';
	txPacket[7] =  '9';
	txPacket[8] =  '9';
	txPacket[9] =  ' ';
	txPacket[10] = 'Y';
	txPacket[11] = 'a';
	txPacket[12] = 'w';
	txPacket[13] = ':';
	txPacket[14] = '9';
	txPacket[15] = '9';
	txPacket[16] = '9';
	txPacket[17] = ' ';
	txPacket[18] = 'R';
	txPacket[19] = 'o';
	txPacket[20] = 'l';
	txPacket[21] = 'l';
	txPacket[22] = ':';
	txPacket[23] = '9';
	txPacket[24] = '9';
	txPacket[25] = '9';
	txPacket[26] = ' ';
	txPacket[27] = ' ';
	txPacket[28] = ' ';
	txPacket[29] = ' ';
	txPacket[30] = '\r';
	txPacket[31] = '\n';
	placePacket(txPacket);
	initSysTimer();

	sei();

	for(;;) {
		
		//From I2C - MPU9250
		uint16_t mpu[7]; //accelX, accelY, accelZ, temp, gyroX, gyroY, gyroZ
		uint8_t tempH, tempL;
	
		startI2CMaster();
		setI2CMaster(I2C_MPU_ID,I2C_M_MODE_WRITE);
		writeI2CMaster(I2C_MPU_ADDR_DATA);
		startI2CMaster();
		setI2CMaster(I2C_MPU_ID,I2C_M_MODE_READ);
	
		for (uint8_t i = 0; i < 4; i++) {
			readI2CMaster(&tempH,I2C_M_RETURN_ACK);
			readI2CMaster(&tempL,I2C_M_RETURN_ACK);
			mpu[i] = (tempH << 8) | tempL;
		}
	
		readI2CMaster(&tempH,I2C_M_RETURN_NAK); //Read into void and return NACK to terminate I2C
		stopI2CMaster();

		//AP control
		float accelX = (float)((int16_t)mpu[0]);
		float accelY = (float)((int16_t)mpu[1]);
		float accelZ = (float)((int16_t)mpu[2]);
		
		//Calculate pitch
		float pitch = -atan( accelX / accelZ ) / M_PI * 180.0; //tan-1(x-axis/z-axis) to degree, range -90 to 90
		uint16_t pitchInt = (uint16_t)pitch + 90;
		
		txPacket[6] = pitchInt / 100 % 10 + '0';
		txPacket[7] = pitchInt / 10 % 10 + '0';
		txPacket[8] = pitchInt % 10 + '0';
		
		//Calculate roll
		float roll = -atan( accelY / accelZ ) / M_PI * 180.0; //tan-1(y-axis/z-axis) to degree, range -90 to 90
		uint16_t rollInt = (uint16_t)roll + 90;
		
		txPacket[23] = rollInt / 100 % 10 + '0';
		txPacket[24] = rollInt / 10 % 10 + '0';
		txPacket[25] = rollInt % 10 + '0';
		
		//Send packet
		placePacket(txPacket);

		//Platform control
		setPPM0(roll / -50.0);
		setPPM1(pitch / 50.0);
		setPPM2(0);
		setPPM3(0);

		//Performance analysis
		PIND = (1<<2); //System time requirement = scope period / 2
	}
	
	return 0;
}


/************************************************************************/
/* Known issues                                                         */
/************************************************************************/