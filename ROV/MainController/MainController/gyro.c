// MPU-6050 Gyro+Accelerometer util

struct mpu {
	int16_t accelX, accelY, accelZ;
	uint16_t temperature;
	int16_t gyroX, gyroY, gyroZ;
};
typedef struct mpu MPU;

#define I2C_MPU_ID 0b1101000
#define I2C_MPU_ADDR_WHOAMI 0x75
#define I2C_MPU_CONTENT_WHOAMI 0x68 //MPU6050
#define I2C_MPU_ADDR_PWRMAGMT1 0x6B
#define I2C_MPU_ADDR_SIGNALPATHRESET 0x68
#define I2C_MPU_ADDR_SLV0CTR 0x25
#define I2C_MPU_ADDR_DATA 0x3B //Addr of first data
#define I2C_MPU_AMOUNT_DATA 14 //6 axis + 1 temperature, 2 bytes each

void writeMPU(uint8_t addr, uint8_t data);
uint8_t readMPU(uint8_t addr);
void readMPUSetAddr(uint8_t addr);
uint8_t readMPUContinue();
uint8_t readMPUEnd();

// Init MPU hardware (reset module and signal path)
void initMPU() {
	writeMPU(I2C_MPU_ADDR_PWRMAGMT1,0x00);
	writeMPU(I2C_MPU_ADDR_SIGNALPATHRESET,0x07);
}

// Read all data (Accel XYZ, Temperature, Gyro XYZ, 7 * 2 bytes)
void readMPUAll(MPU *m) {
	readMPUSetAddr(I2C_MPU_ADDR_DATA);
	m->accelX	= (readMPUContinue()<<8) | readMPUContinue(); //MPU is big-endian
	m->accelY	= (readMPUContinue()<<8) | readMPUContinue();
	m->accelZ	= (readMPUContinue()<<8) | readMPUContinue();
	m->temperature	= (readMPUContinue()<<8) | readMPUContinue();
	m->gyroX	= (readMPUContinue()<<8) | readMPUContinue();
	m->gyroY	= (readMPUContinue()<<8) | readMPUContinue();
	m->gyroZ	= (readMPUContinue()<<8) | readMPUEnd();
}

// I2C util functions
void writeMPU(uint8_t addr, uint8_t data) {
	startI2CMaster();
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_WRITE);
	writeI2CMaster(addr);
	writeI2CMaster(data);
	stopI2CMaster();
}

uint8_t readMPU(uint8_t addr) {
	readMPUSetAddr(addr);
	return readMPUEnd();
}
void readMPUSetAddr(uint8_t addr) {
	startI2CMaster();
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_WRITE);
	writeI2CMaster(addr);
	startI2CMaster();
	setI2CMaster(I2C_MPU_ID,I2C_M_MODE_READ);
}
uint8_t readMPUContinue() {
	uint8_t data;
	readI2CMaster(&data,I2C_M_RETURN_ACK);
	return data;
}
uint8_t readMPUEnd() {
	uint8_t data;
	readI2CMaster(&data,I2C_M_RETURN_NAK);  //Return NAK to terminate I2C
	stopI2CMaster();
	return data;
}
