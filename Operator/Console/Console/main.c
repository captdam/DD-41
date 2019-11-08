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

#define AP_MAX_DEPTH	5000		//in cm, step 10 cm

#define CONFIG_MENUSIZE 10

#define OSD_LINE_PROJECTNAME 0
#define OSD_LINE_APIND 0
#define OSD_LINE_BATTERY 1
#define OSD_LINE_DEPTH 2
#define OSD_LINE_PITCH 3
#define OSD_LINE_COMPASS 4
#define OSD_LINE_JOYSTICK 10
#define OSD_LINE_CONFIG 11


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
#include "../../../AVR_Common/spi.c"

// Functional util
#include "osd.c"
#include "../../../AVR_Common/string.c"



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
	for (uint8_t i = 0; i < 25; i++) _delay_loop_2(65535); //Waiting for external device booting

	PORTB |= (1<<2) | (1<<1); //CS = 1, non-selected
	
	//Init pref
	initUART();
	initADC();
	initSPIMaster();

	//Init OSD module
	initOSD();
	clearOSD();
	
	uint8_t projectName[] = "DD-41";
	stringEncodeOSD(projectName);
	writeSringOSD(OSD_LINE_PROJECTNAME, 0, projectName);
	
	//Init communication system
	uint8_t txPacket[32], rxPacket[32];
	txPacket[31] = pgm_read_word(&( SOFTWARE_INFO[0] ));
	initSysTimer();

	//System task memory (keep after each loop)
	uint8_t digitalInputLast = 0x00; //Software PCINT
	uint8_t menuIndex = 0; //Config menu (menu element)
	
	uint8_t motorPowerIndex[4] = {100, 100, 100, 100}; //Engine output = joystick * this (range 0 to 100)
	
	uint8_t apEnable = 0; //Auto pilot enable/disable
	uint16_t apDepth = 0; //Range 0 to AP_MAX_DEPTH, in cm
	uint16_t apCompass = 0; //Range 0 to 359, in degree
	int8_t apPitch = 0; //Range +/- 89, in degree

	uint8_t functionMainLight = 0, functionNaviLight = 0;

	//System task start
	sei();

	//System task loop
	for (;;) {
		
		/************************************************************************/
		/* 1 - Get ROV data                                                     */
		/************************************************************************/

		fetchPacket(rxPacket);
		uint16_t depth =		(rxPacket[COM_PACKET_DATA_DEPTH_H]<<8) | rxPacket[COM_PACKET_DATA_DEPTH_L];
		uint16_t pitch =		(rxPacket[COM_PACKET_DATA_PITCH_H]<<8) | rxPacket[COM_PACKET_DATA_PITCH_L];
		uint16_t compass =		(rxPacket[COM_PACKET_DATA_COMPASS_H]<<8) | rxPacket[COM_PACKET_DATA_COMPASS_L];
		uint16_t temperature =		(rxPacket[COM_PACKET_DATA_TEMPERATURE_H]<<8) | rxPacket[COM_PACKET_DATA_TEMPERATURE_L];
		uint16_t voltage =		(rxPacket[COM_PACKET_DATA_VOLTAGE_H]<<8) | rxPacket[COM_PACKET_DATA_VOLTAGE_L];

		/************************************************************************/
		/* 2 - Display ROV data on OSD                                          */
		/************************************************************************/

		//Display ROV battery voltage
		uint8_t displayVoltage[] = "ROV Bat --.--V";
		stringEncodeOSD(displayVoltage);

		uint16_t displayVoltageHigh = bcd2osd( voltage >> 8 );
		displayVoltage[8] = displayVoltageHigh >> 8;
		displayVoltage[9] = displayVoltageHigh & 0xFF;

		uint16_t displayVoltageLow = bcd2osd( voltage & 0xFF );
		displayVoltage[11] = displayVoltageLow >> 8;
		displayVoltage[12] = displayVoltageLow & 0xFF;
		writeSringOSD(OSD_LINE_BATTERY, 0, displayVoltage);
		
		//Display ROV depth
		uint8_t displayDepth[] = "Depth --.--m";
		stringEncodeOSD(displayDepth);
		
		uint16_t displayDepthHigh = bcd2osd( depth >> 8 );
		displayDepth[6] = displayDepthHigh >> 8;
		displayDepth[7] = displayDepthHigh & 0xFF;
		
		uint16_t displayDepthLow = bcd2osd( depth & 0xFF );
		displayDepth[9] = displayDepthLow >> 8;
		displayDepth[10] = displayDepthLow & 0xFF;
		writeSringOSD(OSD_LINE_DEPTH, 0, displayDepth);
		
		/************************************************************************/
		/* 3.1 - Get user input (joystick)                                      */
		/************************************************************************/

		uint16_t joystick[4];
		uint8_t displayJoystick[20];
		for (uint8_t i = 0; i < 4; i++) {
			joystick[i] = getADC(i);

			////////////////////////////////////////////////////////////////////////// For joystick curve dev use
			uint16_t displayJoystickTempBCD, displayJoystickTempOSD;
			displayJoystickTempBCD = uint2bcd(joystick[i]);
			displayJoystickTempOSD = bcd2osd( displayJoystickTempBCD >> 8 );
			displayJoystick[ i * 5     ] = displayJoystickTempOSD >> 8;
			displayJoystick[ i * 5 + 1 ] = displayJoystickTempOSD & 0xFF;
			displayJoystickTempOSD = bcd2osd( displayJoystickTempBCD & 0xFF );
			displayJoystick[ i * 5 + 2 ] = displayJoystickTempOSD >> 8;
			displayJoystick[ i * 5 + 3 ] = displayJoystickTempOSD & 0xFF;
		}
		displayJoystick[19] = 0xFF;
		writeSringOSD(OSD_LINE_JOYSTICK, 0, displayJoystick);

		/************************************************************************/
		/* 3.2 - Get user input (press key)                                     */
		/************************************************************************/

		//Scan user input
		int8_t menuAction = 0;
		uint8_t digitalInput = (~PIND & 0b11111100);
		if (digitalInputLast != digitalInput) { //Key pressed or released (Pin Change)
			switch (digitalInput) {
				case (1<<7): menuIndex++;      break; //Menu +
				case (1<<6): menuAction =  10; break; //Value +10
				case (1<<5): menuAction =  1;  break; //Value +
				case (1<<4): menuAction = -1;  break; //Value -
				case (1<<3): menuAction = -10; break; //Value -10
				case (1<<2): menuIndex--;      break; //Menu -
				default:                       break; //Multiple key is not allowed
			}
		}
		digitalInputLast = digitalInput;
		
		//Display current menu element and value under that element
		if (menuIndex == 0xFF)
			menuIndex += CONFIG_MENUSIZE;
		if (menuIndex == CONFIG_MENUSIZE)
			menuIndex = 0;

		// 0 - A/P enable
		if (!menuIndex) {
			//If any key pressed, toggle
			if (menuAction)
				apEnable = ( apEnable + 1 ) & 1; 
				
			if (apEnable) {
				uint8_t lineMenu[30] = "A/P Ctrl ON                  ";
				stringEncodeOSD(lineMenu);
				writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);

				uint8_t apline[30] = "A/P Activated";
				stringEncodeOSD(apline);
				writeSringBlinkOSD(OSD_LINE_APIND, 15, apline);
			}
			else {
				uint8_t lineMenu[30] = "A/P Ctrl OFF                 ";
				stringEncodeOSD(lineMenu);
				writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);

				uint8_t apline[30] = "A/P Idle     ";
				stringEncodeOSD(apline);
				writeSringOSD(OSD_LINE_APIND, 15, apline);
			}
		}

		// 1 - A/P Depth, step = 0.10m
		else if (menuIndex == 1) {
			apDepth += menuAction * 10;
			if (apDepth > AP_MAX_DEPTH && apDepth < AP_MAX_DEPTH + 500) //Too deep
				apDepth = AP_MAX_DEPTH;
			else if (apDepth > AP_MAX_DEPTH) //Unsigned int downflow
				apDepth = 0;

			uint8_t lineMenu[30] = "A/P Depth --.--m             ";
			stringEncodeOSD(lineMenu);
			uint16_t apDepthBCD = uint2bcd(apDepth);
			
			uint16_t apDepthDisplayHigh = bcd2osd(apDepthBCD >> 8);
			lineMenu[10] = apDepthDisplayHigh >> 8;
			lineMenu[11] = apDepthDisplayHigh & 0xFF;
			
			uint16_t apDepthDisplayLow = bcd2osd(apDepthBCD & 0xFF);
			lineMenu[13] = apDepthDisplayLow >> 8;
			lineMenu[14] = apDepthDisplayLow & 0xFF;
			writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
		}

		// 2 - A/P Pitch
		else if (menuIndex == 2) {
			apPitch += menuAction;
			if (apPitch > 89)
				apPitch = 89;
			else if (apPitch < -89)
				apPitch = -89;

			uint8_t lineMenu[30] = "A/P Pitch --' ----           ";
			stringEncodeOSD(lineMenu);

			if (apPitch == 0) { //"00' FLAT"
				lineMenu[10] = 0x0A;
				lineMenu[11] = 0x0A;
				lineMenu[14] = 0x10;
				lineMenu[15] = 0x16;
				lineMenu[16] = 0x0B;
				lineMenu[17] = 0x1E;
			}
			else if (apPitch > 0) { //"xx' UP  "
				uint8_t diffBCD = uint2bcd( (uint8_t)apPitch );
				uint16_t diffOSD = bcd2osd(diffBCD);
				lineMenu[10] = diffOSD >> 8;
				lineMenu[11] = diffOSD & 0xFF;
				lineMenu[14] = 0x1F;
				lineMenu[15] = 0x1A;
				lineMenu[16] = 0x00;
				lineMenu[17] = 0x00;
			}
			else { //"xx' DOWN"
				uint8_t diffBCD = uint2bcd( (uint8_t)(-apPitch) );
				uint16_t diffOSD = bcd2osd(diffBCD);
				lineMenu[10] = diffOSD >> 8;
				lineMenu[11] = diffOSD & 0xFF;
				lineMenu[14] = 0x0E;
				lineMenu[15] = 0x19;
				lineMenu[16] = 0x21;
				lineMenu[17] = 0x18;
			}

			writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
		}

		// 3 - A/P Compass
		else if (menuIndex == 3) {
			apCompass += menuAction;
			if (apCompass > 359 && apCompass < 600) //Overflow
				apCompass -= 360;
			else if (apCompass > 359) //Downflow
				apCompass += 360;

			uint8_t lineMenu[30] = "A/P Compass ---' -           ";
			stringEncodeOSD(lineMenu);

			uint16_t apCompassBCD = uint2bcd(apCompass);
			uint8_t apCompassHighOSD = bcd2osd(apCompassBCD >> 8);
			uint16_t apCompassLowOSD = bcd2osd(apCompassBCD & 0xFF);
			lineMenu[12] = apCompassHighOSD;
			lineMenu[13] = apCompassLowOSD >> 8;
			lineMenu[14] = apCompassLowOSD & 0xFF;
			
			if	(apCompass >  45 && apCompass <= 135)	lineMenu[17] = 0x0F; //"E"
			else if	(apCompass > 135 && apCompass <= 225)	lineMenu[17] = 0x1D; //"S"
			else if	(apCompass > 225 && apCompass <= 315)	lineMenu[17] = 0x21; //"W"
			else						lineMenu[17] = 0x18; //"N"
			
			writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
		}

		// 4~7 - Motor power index
		else if ( menuIndex >= 4 && menuIndex <= 7 ) {
			uint8_t mi =  menuIndex - 4;
			motorPowerIndex[mi] += menuAction;
			
			if ( motorPowerIndex[mi] > 100 && motorPowerIndex[mi] < 200 ) //Overflow
				motorPowerIndex[mi] = 100;
			else if ( motorPowerIndex[mi] > 100) //Downflow
				motorPowerIndex[mi] = 0;

			uint8_t lineMenu[30] = "Mtr P Idx - ---              ";
			stringEncodeOSD(lineMenu);

			uint8_t motorIndexOSD = bcd2osd(mi);
			uint16_t motorPowerIndexBCD = uint2bcd(motorPowerIndex[mi]);
			uint8_t motorPowerIndexHighOSD = bcd2osd(motorPowerIndexBCD >> 8);
			uint16_t motorPowerIndexLowOSD = bcd2osd(motorPowerIndexBCD & 0xFF);
			lineMenu[10] = motorIndexOSD;
			lineMenu[12] = motorPowerIndexHighOSD;
			lineMenu[13] = motorPowerIndexLowOSD >> 8;
			lineMenu[14] = motorPowerIndexLowOSD & 0xFF;
			writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
		}

		// 8 - Function - main lights
		else if (menuIndex == 8) {
			//If any key pressed, toggle
			if (menuAction)
				functionMainLight = ( functionMainLight + 1 ) & 1; 
				
			if (functionMainLight) {
				uint8_t lineMenu[30] = "F M Light ON                 ";
				stringEncodeOSD(lineMenu);
				writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
			}
			else {
				uint8_t lineMenu[30] = "F M Light OFF                ";
				stringEncodeOSD(lineMenu);
				writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
			}
		}

		// 9 - Function - navi light
		else {
			if (menuAction)
				functionNaviLight = ( functionNaviLight + 1 ) & 1; 
				
			if (functionNaviLight) {
				uint8_t lineMenu[30] = "F Nav Light ON               ";
				stringEncodeOSD(lineMenu);
				writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
			}
			else {
				uint8_t lineMenu[30] = "F Nav Light OFF              ";
				stringEncodeOSD(lineMenu);
				writeSringOSD(OSD_LINE_CONFIG, 0, lineMenu);
			}
		}

		/************************************************************************/
		/* 6 - End of system task loop, place Tx packet                         */
		/************************************************************************/
		txPacket[COM_PACKET_CTRL_JOYSTICK0_L] = joystick[0] & 0xFF;
		txPacket[COM_PACKET_CTRL_JOYSTICK0_H] = joystick[0] >> 8;
		txPacket[COM_PACKET_CTRL_JOYSTICK1_L] = joystick[1] & 0xFF;
		txPacket[COM_PACKET_CTRL_JOYSTICK1_H] = joystick[1] >> 8;
		txPacket[COM_PACKET_CTRL_JOYSTICK2_L] = joystick[2] & 0xFF;
		txPacket[COM_PACKET_CTRL_JOYSTICK2_H] = joystick[2] >> 8;
		txPacket[COM_PACKET_CTRL_JOYSTICK3_L] = joystick[3] & 0xFF;
		txPacket[COM_PACKET_CTRL_JOYSTICK3_H] = joystick[3] >> 8;
		
		txPacket[COM_PACKET_CTRL_MTRPOW0] = motorPowerIndex[0];
		txPacket[COM_PACKET_CTRL_MTRPOW1] = motorPowerIndex[1];
		txPacket[COM_PACKET_CTRL_MTRPOW2] = motorPowerIndex[2];
		txPacket[COM_PACKET_CTRL_MTRPOW3] = motorPowerIndex[3];

		
		txPacket[COM_PACKET_CTRL_FUNCTION] =	( ( apEnable		&1) << COM_PACKET_CTRL_FUNCTION_AP )
						|	( ( functionMainLight	&1) << COM_PACKET_CTRL_FUNCTION_ML )
						|	( ( functionNaviLight	&1) << COM_PACKET_CTRL_FUNCTION_NL );
						
		txPacket[COM_PACKET_CTRL_AP_PITCH] = apPitch;
		txPacket[COM_PACKET_CTRL_AP_COMPASS_L] = apCompass & 0xFF;
		txPacket[COM_PACKET_CTRL_AP_COMPASS_L] = apCompass >> 8;
		txPacket[COM_PACKET_CTRL_AP_DEPTH_H] = apDepth & 0xFF;
		txPacket[COM_PACKET_CTRL_AP_DEPTH_H] = apDepth >> 8;

		placePacket(txPacket);
		PINB = (1<<0); //System status output
	}
	
	return 0;
}


/************************************************************************/
/* Known issues                                                         */
/************************************************************************/