#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <math.h>
#include <util/delay_basic.h>

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

// Base util
#include "../../../Communication/Software/communication.c"
#include "../../../AVR_Common/adc.c"
#include "../../../AVR_Common/twi.c"

// Function util
#include "ppm.c"
#include "gyro.c"
#include "pid.c"


// MAIN ROUTINES ---------------------------------------------------------

int main() {
	//Init I/O
	DDRD = 0xFF; //Digital out
	DDRC = 0x00; //ADC
	DDRB = 0xFF; //SPI CS, System status output
	for (uint8_t i = 0; i < 25; i++) _delay_loop_2(65535); //Waiting for external device booting

	//Init pref
	initUART();
	initADC();
	initI2CMaster(40000);

	initPPM(); //Init PPM output (servo control)
	initMPU(); //Init MPU (Gyro)

	//Init communication system
	uint8_t txPacket[32], rxPacket[32];
	txPacket[31] = pgm_read_word(&( SOFTWARE_INFO[0] ));
	initSysTimer();

	//System task memory

	//System task start
	sei();

	//System task loop
	for (;;) {
		
		/************************************************************************/
		/* 1 - Get ROV status                                                   */
		/************************************************************************/
		
		//Read from MPU
		MPU motionStatus;
		readMPUAll(&motionStatus);
		
		//Calculate pitch and roll
		float accelX = (float)motionStatus.accelX;
		float accelY = (float)motionStatus.accelY;
		float accelZ = (float)motionStatus.accelZ;
		
		float pitch = -atan( accelY / accelZ ) / M_PI * 180.0; //tan-1(x-axis/z-axis) to degree, range -90 to 90
//		float roll = atan( accelX / accelZ ) / M_PI * 180.0; //tan-1(y-axis/z-axis) to degree, range -90 to 90
		
		//Read from ADC
		uint16_t adc0 = getADC(0);
		uint16_t adc1 = getADC(1);
		uint16_t adc2 = getADC(2);
		uint16_t adc3 = getADC(3);

		uint16_t batteryVoltage = (uint16_t)( (float)adc0 / 1023 * 5 * 3 * 100 );
		uint16_t depth = (uint16_t)( (float)adc1 / 1023 * 4000 );


		/************************************************************************/
		/* 2 - Get user command                                                 */
		/************************************************************************/

		fetchPacket(rxPacket);
		uint16_t userAxis0 = (rxPacket[COM_PACKET_CTRL_JOYSTICK0_H]<<8) | rxPacket[COM_PACKET_CTRL_JOYSTICK0_L]; //Joystick, range 0-1023
		uint16_t userAxis1 = (rxPacket[COM_PACKET_CTRL_JOYSTICK1_H]<<8) | rxPacket[COM_PACKET_CTRL_JOYSTICK1_L];
		uint16_t userAxis2 = (rxPacket[COM_PACKET_CTRL_JOYSTICK2_H]<<8) | rxPacket[COM_PACKET_CTRL_JOYSTICK2_L];
		uint16_t userAxis3 = (rxPacket[COM_PACKET_CTRL_JOYSTICK3_H]<<8) | rxPacket[COM_PACKET_CTRL_JOYSTICK3_L];
		uint8_t userAxisPwr0 = rxPacket[COM_PACKET_CTRL_MTRPOW0]; //Power index, % of max for each axis, NOT USED
		uint8_t userAxisPwr1 = rxPacket[COM_PACKET_CTRL_MTRPOW1];
		uint8_t userAxisPwr2 = rxPacket[COM_PACKET_CTRL_MTRPOW2];
		uint8_t userAxisPwr3 = rxPacket[COM_PACKET_CTRL_MTRPOW3];
		uint8_t userFunctionAP = (rxPacket[COM_PACKET_CTRL_MTRPOW3]>>COM_PACKET_CTRL_FUNCTION_AP) & 1;
		uint8_t userFunctionML = (rxPacket[COM_PACKET_CTRL_MTRPOW3]>>COM_PACKET_CTRL_FUNCTION_ML) & 1;
		uint8_t userFunctionNL = (rxPacket[COM_PACKET_CTRL_MTRPOW3]>>COM_PACKET_CTRL_FUNCTION_NL) & 1;
		int8_t userPitchAP = rxPacket[COM_PACKET_CTRL_AP_PITCH];
		uint16_t userCompassAP = (rxPacket[COM_PACKET_CTRL_AP_COMPASS_H]<<8) | rxPacket[COM_PACKET_CTRL_AP_COMPASS_L];
		uint16_t userDepthAP = (rxPacket[COM_PACKET_CTRL_AP_DEPTH_H]<<8) | rxPacket[COM_PACKET_CTRL_AP_DEPTH_L];

		/************************************************************************/
		/* 3 - Process user command w/ AP control                               */
		/************************************************************************/

		//Analysis user input axis info (joystick)
		uint16_t userForward = userAxis0;
		uint16_t userRight = userAxis1;
		uint16_t userUp = userAxis2;
		uint16_t userPitchUp = userAxis3;
		
		float servoLeft = (float)(userForward + userRight - 0x200) / 0x400; //For each axis, range -0.5 ~ 1.5, stay center if 0.5
		float servoRight = (float)(userForward - userRight + 0x200) / 0x400;
		float servoHead = (float)(userUp + userPitchUp - 0x200) / 0x400;
		float servoTail = (float)(userUp - userPitchUp + 0x200) / 0x400;

		servoLeft -= 0.5; //Normalize to +/-1.0 range, stay center if 0
		servoRight -= 0.5;
		servoHead -= 0.5;
		servoTail -= 0.5;

		//Auto-pilot
		if (userFunctionAP) {

		}

		//Misc functions
		uint8_t digitalOutput =	( userFunctionML	<< 0 )
				|	( userFunctionNL	<< 1 );


		/************************************************************************/
		/* 4 - Output                                                           */
		/************************************************************************/
		
		//Servo control
		setPPM0(pitch / 90.0);
		setPPM1(pitch / 90.0);
		setPPM2(pitch / 90.0);
		setPPM3(pitch / 90.0);

		/************************************************************************/
		/* 5 - Update Tx buffer according to ROV status                         */
		/************************************************************************/
		
		txPacket[COM_PACKET_DATA_DEPTH_L]	= depth & 0xFF;
		txPacket[COM_PACKET_DATA_DEPTH_H]	= depth >> 8;
		txPacket[COM_PACKET_DATA_PITCH_L]	= (int16_t)pitch & 0xFF;
		txPacket[COM_PACKET_DATA_PITCH_H]	= (int16_t)pitch >> 8;
		txPacket[COM_PACKET_DATA_COMPASS_L]	= 0;
		txPacket[COM_PACKET_DATA_COMPASS_H]	= 0;
		txPacket[COM_PACKET_DATA_TEMPERATURE_L]	= 0;
		txPacket[COM_PACKET_DATA_TEMPERATURE_H]	= 0;
		txPacket[COM_PACKET_DATA_VOLTAGE_L]	= batteryVoltage & 0xFF;
		txPacket[COM_PACKET_DATA_VOLTAGE_H]	= batteryVoltage >> 8;

		txPacket[COM_PACKET_DATA_MPU_AXL]	= motionStatus.accelX & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_AXH]	= motionStatus.accelX >> 8;
		txPacket[COM_PACKET_DATA_MPU_AYL]	= motionStatus.accelY & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_AYH]	= motionStatus.accelY >> 8;
		txPacket[COM_PACKET_DATA_MPU_AZL]	= motionStatus.accelZ & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_AZH]	= motionStatus.accelZ >> 8;
		txPacket[COM_PACKET_DATA_MPU_TEMPL]	= motionStatus.temperature & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_TEMPH]	= motionStatus.temperature >> 8;
		txPacket[COM_PACKET_DATA_MPU_GXL]	= motionStatus.gyroX & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_GXH]	= motionStatus.gyroX >> 8;
		txPacket[COM_PACKET_DATA_MPU_GYL]	= motionStatus.gyroY & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_GYH]	= motionStatus.gyroY >> 8;
		txPacket[COM_PACKET_DATA_MPU_GZL]	= motionStatus.gyroZ & 0xFF;
		txPacket[COM_PACKET_DATA_MPU_GZH]	= motionStatus.gyroZ >> 8;
		
		txPacket[COM_PACKET_DATA_ADC0]		= adc0;
		txPacket[COM_PACKET_DATA_ADC1]		= adc1;
		txPacket[COM_PACKET_DATA_ADC2]		= adc2;
		txPacket[COM_PACKET_DATA_ADC3]		= adc3;

		/************************************************************************/
		/* 6 - End of system task loop, place Tx packet                         */
		/************************************************************************/

		placePacket(txPacket);
		PINB = (1<<0); //System status output
	}
	
	return 0;
}


/************************************************************************/
/* Known issues                                                         */
/************************************************************************/