#include "MLX90363.h"


/**
 * Array for calculating CRC values, courtesy of Melexis.
 */
static uint8_t CRCArray[] = {
0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26,
0xEB, 0xC4, 0xB5, 0x9A, 0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63,
0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34, 0x73, 0x5C, 0x2D, 0x02,
0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB,
0x36, 0x19, 0x68, 0x47, 0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B,
0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C, 0x48, 0x67, 0x16, 0x39,
0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3,
0x7E, 0x51, 0x20, 0x0F, 0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6,
0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1, 0xE3, 0xCC, 0xBD, 0x92,
0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B,
0xA6, 0x89, 0xF8, 0xD7, 0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D,
0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A, 0x3E, 0x11, 0x60, 0x4F,
0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23,
0xEE, 0xC1, 0xB0, 0x9F, 0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66,
0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31, 0x76, 0x59, 0x28, 0x07,
0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE,
0x33, 0x1C, 0x6D, 0x42 };



/**
 * Array for determining EEPROM Write Keys.
 * (Page 38, Table 39)
 * @note I might not be allowed to put this here? (Page 38, Protection A)
 */
static uint16_t writeKeys[4][8] = {
		{17485, 31053, 57190, 57724, 7899,  53543, 26763, 12528} ,
		{38105, 51302, 16209, 24847, 13134, 52339, 14530, 18350},
		{55636, 64477, 40905, 45498, 24411, 36677, 4213,  48843} ,
		{6368,   5907, 31384, 63325, 3562,  19816, 6995,  3147}
};


/**
 * Clears all the buffers, duh.
 * Buffer clearing:
 */
void MLX90363_clearBuffers(MLX90363* dev){
	for(int i = 0; i < 8; ++i){
		dev->rxBuffer[i] = 0;
		dev->txBuffer[i] = 0;
	}
}

/**
 * Initialization:
 * Sets up SPI and MLX struct fields.
 * @return maybe some error code -- im thinknig the deviceID lookup can tell us if the bus is working.
 */
uint8_t MLX90363_init(SPI_HandleTypeDef* spi_h, MLX90363* dev){
	dev->spi_h = spi_h;
	MLX90363_clearBuffers(dev);
	//TODO: add functionality for finding the device ID (this includes sending an eeprom read message)
	return(123); // CHANGE LATER
}

/**
 * CRC Calculation:
 * Encodes or Decodes CRC value, given an 8byte buffer.
 * @return Value of CRC, or -1 if theres some evil stuff (user gives incompatible array maybe?)
 */
uint8_t computeCRC(uint8_t* buffer){
	uint8_t crc = 0xFF;
	crc = CRCArray[ buffer[0] ^ crc ];
	crc = CRCArray[ buffer[1] ^ crc ];
	crc = CRCArray[ buffer[2] ^ crc ];
	crc = CRCArray[ buffer[3] ^ crc ];
	crc = CRCArray[ buffer[4] ^ crc ];
	crc = CRCArray[ buffer[5] ^ crc ];
	crc = CRCArray[ buffer[6] ^ crc ];
	crc = ~crc;
	//TODO: Implement a check for evil case maybe?
	return crc;
}

/**
 * CRC Verification:
 * @return 1 if the CRC matches, 0 if not.
 */
uint8_t verifyCRC(MLX90363* dev){
	// CRC is stored in 8th byte, so we just compare that with the expected value.
	uint8_t devCRC 	= dev->rxBuffer[7];
	uint8_t myCRC	= computeCRC(dev->rxBuffer);
	return(myCRC == devCRC);
}


/**
 * Memory Read
 * Helper function for reading two EEPROM or RAM words.
 * I think I'll have another function with a switch statment or something idek.
 * @param ADDRx The EEPROM or RAM address you want to read from. Note: I don't really know why we would care about the RAM contents, but EEPROM is useful, as alot of parameters are stored there.
 * @return error code?
 */

uint8_t MLX90363_memRead(MLX90363* dev, uint16_t ADDR0, uint16_t ADDR1){
	dev->txBuffer[0] = (ADDR0 & 0xFF);	// low byte of address 0
	dev->txBuffer[1] = (ADDR0 >> 8);	// high byte of address 0
	dev->txBuffer[2] = (ADDR1 & 0xFF);
	dev->txBuffer[3] = (ADDR1 >> 8);
	dev->txBuffer[4] = 0x00;
	dev->txBuffer[5] = 0x00;
	dev->txBuffer[6] = Other | MemoryRead;
	dev->txBuffer[7] = computeCRC(dev->txBuffer);

	nCS_L;
	HAL_SPI_TransmitReceive(dev->spi_h, dev->txBuffer, dev->rxBuffer, sizeof(dev->txBuffer), HAL_MAX_DELAY);
	nCS_H;

	return 1; // CHANGE
}


/**
 * EEPROM Write / memWrite
 * @param ADDR, the address you want to write to.
 * @param data, the data you want to write into the EEPROM.
 * @note The address must be even. (pg 37, 13.17, note (16), rev 006)
 * @return error code?
 * TODO: TEST THIS
 * idea, put this into the body of a memWrite function, which will take in the EEPROM messages enum.
 */
uint8_t MLX90363_EEWrite(MLX90363* dev, uint8_t ADDR, uint16_t data){
	// check if even here, maybe?

	// determine key
	uint8_t keyCol = (ADDR >> 1) & 0x07; // ADDR[3:1]
	uint8_t keyRow = (ADDR >> 4) & 0x03; // ADDR[5:4]

	uint16_t key = writeKeys[keyRow][keyCol];

	dev->txBuffer[0] = 0;
	dev->txBuffer[1] = ADDR & 0x3F;
	dev->txBuffer[2] = key & 0xFF;
	dev->txBuffer[3] = key >> 8;
	dev->txBuffer[4] = data & 0xFF; // low byte of data
	dev->txBuffer[5] = data >> 8;   // high byte
	dev->txBuffer[6] = Other | EEWrite;
	dev->txBuffer[7] = computeCRC(dev->txBuffer);

	nCS_L;
	HAL_SPI_TransmitReceive(dev->spi_h, dev->txBuffer, dev->rxBuffer, sizeof(dev->txBuffer), HAL_MAX_DELAY);
	nCS_H;

	return 1; // CHANGE

}




/**
 * NOP Instruction
 * A tool for debugging, "clocking", and general use of the SPI bus.
 * @param key Any number you want. The use case is to check if the received message makes sense. (first nop recieves a junk message, second instruction, implied ot be another nop, will return a nop but with your key, as well the inversion of your key.
 * @return Error code maybe? TODO: figure this out.
 */
uint8_t MLX90363_NOP(MLX90363* dev, uint16_t key){
	dev->txBuffer[0] = 0x00;
	dev->txBuffer[1] = 0x00;
	dev->txBuffer[2] = key & 0xFF;
	dev->txBuffer[3] = key >> 8;
	dev->txBuffer[4] = 0x00;
	dev->txBuffer[5] = 0x00;
	dev->txBuffer[6] = Other | NOP_Challenge;
	dev->txBuffer[7] = computeCRC(dev->txBuffer);

	nCS_L;
	HAL_SPI_TransmitReceive(dev->spi_h, dev->txBuffer, dev->rxBuffer, sizeof(dev->txBuffer), HAL_MAX_DELAY);
	nCS_H;

	return 1; // CHANGE

}

/**
 * GET1 Instruction
 * Instruction for recieving regular data message
 * @return See above function
 */
uint8_t MLX90363_GET1(MLX90363* dev, MessageType msgType, uint16_t timeout){
	dev->txBuffer[0] = 0x00;
	dev->txBuffer[1] = 0x00; // This should be RST, the bit that resets the rolling counter. Don't see how that useful so far, so I'm just gonna keep it at 0.
	dev->txBuffer[2] = timeout >> 8; 	// high byte of timeout
	dev->txBuffer[3] = timeout & 0xFF;  // low byte of timeout
	dev->txBuffer[4] = 0x00;
	dev->txBuffer[5] = 0x00;
	dev->txBuffer[6] = msgType | GET1;
	dev->txBuffer[7] = computeCRC(dev->txBuffer);

	nCS_L;
	HAL_SPI_TransmitReceive(dev->spi_h, dev->txBuffer, dev->rxBuffer, sizeof(dev->txBuffer), HAL_MAX_DELAY);
	nCS_H;

	return 1; // CHANGE;

}


/**
 * getAngle
 * Specifically finds the measured angle, instead of making user dig through rxBuffer.
 * @return Rotary Angle currently measured by the sensor.
 */
float MLX90363_getAngle(MLX90363* dev){
	float res;
	uint16_t angle_lsb;
	MLX90363_GET1(dev, Alpha, 0xFFFF);
	HAL_Delay(1);
	MLX90363_NOP(dev, 0xAAAA);

	angle_lsb = (dev->rxBuffer[1] & 0x3F) << 8;
	angle_lsb += dev->rxBuffer[0];

	res = angle_lsb *  LSB14_TO_FLOAT;

	return res;
}
