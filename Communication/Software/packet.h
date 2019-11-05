// Packet structure defination
// Both packet are 32-byte long

// packet[0] will be send first, packet[31] will be the last
// For multi-byte data, higher byte should be contained in packet[higher_index]


/************************************************************************/
/* Control packet ( ROV --> Operator )                                  */
/************************************************************************/

//Joysticks (PPM motor/servo ctrl) -	10-bit uint
#define COM_PACKET_CTRL_JOYSTICK0_L	0
#define COM_PACKET_CTRL_JOYSTICK0_H	1
#define COM_PACKET_CTRL_JOYSTICK1_L	2
#define COM_PACKET_CTRL_JOYSTICK1_H	3
#define COM_PACKET_CTRL_JOYSTICK2_L	4
#define COM_PACKET_CTRL_JOYSTICK2_H	5
#define COM_PACKET_CTRL_JOYSTICK3_L	6
#define COM_PACKET_CTRL_JOYSTICK3_H	7

//Motor power index -			0~100 uint
#define COM_PACKET_CTRL_MTRPOW0		8
#define COM_PACKET_CTRL_MTRPOW1		9
#define COM_PACKET_CTRL_MTRPOW2		10
#define COM_PACKET_CTRL_MTRPOW3		11

//Functions -				Mask
#define COM_PACKET_CTRL_FUNCTION	12
#define COM_PACKET_CTRL_FUNCTION_AP	0 //bit shift
#define COM_PACKET_CTRL_FUNCTION_ML	1
#define COM_PACKET_CTRL_FUNCTION_NL	2

//AP - Pitch -				+/-90 int
#define COM_PACKET_CTRL_AP_PITCH	13

//AP - Compass -			0~356 uint
#define COM_PACKET_CTRL_AP_COMPASS_L	14
#define COM_PACKET_CTRL_AP_COMPASS_H	15

//AP - Depth -				16-bit uint (*10)(cm)
#define COM_PACKET_CTRL_AP_DEPTH_L	16
#define COM_PACKET_CTRL_AP_DEPTH_H	17


/************************************************************************/
/* Data packet ( ROV <-- Operator )                                     */
/************************************************************************/

#define COM_PACKET_DATA_DEPTH_L		0
#define COM_PACKET_DATA_DEPTH_H		1
#define COM_PACKET_DATA_PITCH_L		2
#define COM_PACKET_DATA_PITCH_H		3
#define COM_PACKET_DATA_COMPASS_L	4
#define COM_PACKET_DATA_COMPASS_H	5
#define COM_PACKET_DATA_TEMPERATURE_L	6
#define COM_PACKET_DATA_TEMPERATURE_H	7
#define COM_PACKET_DATA_VOLTAGE_L	8
#define COM_PACKET_DATA_VOLTAGE_H	9