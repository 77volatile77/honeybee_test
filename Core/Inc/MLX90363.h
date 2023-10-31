/*
 * MLX90363.h
 *
 *  Created on: Aug 19, 2023
 *      Author: uzairsyed
 */

#ifndef INC_MLX90363_H_
#define INC_MLX90363_H_

#include "stm32l4xx_hal.h"

#define nCS_PORT GPIOA
#define nCS_PIN  GPIO_PIN_4

#define nCS_L		HAL_GPIO_WritePin(nCS_PORT, nCS_PIN, GPIO_PIN_RESET);
#define nCS_H		HAL_GPIO_WritePin(nCS_PORT, nCS_PIN, GPIO_PIN_SET);

#define LSB14_TO_FLOAT	0.0219726562f


/******************* SPI CONFIG PARAMETERS *******************/
//					CPHA=1, CPOL=0, LSBFE=0

/******************* OPCODES *******************/
typedef enum {

	/** Sending Opcodes **/
	GET1 							=	0x13,
	GET2 							= 	0x14,
	GET3 							= 	0x15,
	MemoryRead 						= 	0x01, 	//	for reading 2 EEPROM words (word = u16).
	EEWrite 						= 	0x03,
	EEChallengeAns 					= 	0x05,
	NOP_Challenge 					= 	0x10,
	DiagnosticDetails 				= 	0x16,
	OscCounterStart 				= 	0x18,
	OscCounterStop 					= 	0x1A,
	Reboot 							= 	0x2F,
	Standby 						= 	0x31,

	/** Receiving Opcodes **/
	Get3Ready						= 	0x2D,
	MemoryReadAnswer				= 	0x02,
	EEWriteChallenge				=	0x04,
	EEReadAnswer					=	0x28,
	EEWriteStatus					=	0x0E,
	Challenge_NOP_MISO				=	0x11,
	DiagnosticsAnswer				=	0x17,
	OscCounterStartAck				=	0x19,
	OscCounterStopAck_CounterValue	=	0x1B,
	StandbyAck						=	0x32,
	ErrorFrame						=	0x3D,
	NothingToTransmit				=	0x3E,
	ReadyMessage					=	0x2C

}Opcode;

/******************* MESSAGE TYPES *******************/
typedef enum {
	/** These are the top 2 bits of byte6, the byte that has the opcode in it.
	 * 	They tell the chip what type of message we want to send:
	 * 	Alpha	->	single angle	(rotary)
	 * 	Beta	->	two angles 		(joy-stick)
	 * 	XYZ 	->	raw XYZ components of angles. (See MLX90363 datasheet section 16.1 for more info)
	 */
	Alpha 		= 0,			// We clear the entire byte, so we don't have to use &= ~(3 << 6).
	Alpha_Beta 	= (1 << 6),
	XYZ 		= (2 << 6),
	Other		= (3 << 6)		// The unspecified marker, this can be found in MemoryRead for example.
}MessageType;


/******************* EEPROM MESSAGES *****************/
//typedef enum {
//
//}EEWriteMessages;


/******************* DEVICE STRUCT *****************/
typedef struct{
	SPI_HandleTypeDef* spi_h;
	uint8_t txBuffer[8];
	uint8_t rxBuffer[8];
	uint64_t deviceID;
}MLX90363;

// FUNCTION PROTOTYPES

uint8_t MLX90363_init(SPI_HandleTypeDef* spi_h, MLX90363* dev);
uint8_t MLX90363_memRead(MLX90363* dev, uint16_t ADDR0, uint16_t ADDR1);
uint8_t MLX90363_EEWrite(MLX90363* dev, uint8_t ADDR, uint16_t data);
uint8_t MLX90363_NOP(MLX90363* dev, uint16_t key);
uint8_t MLX90363_GET1(MLX90363* dev, MessageType msgType, uint16_t timeout);
float MLX90363_getAngle(MLX90363* dev);
// TODO: implement eeprom write and read.
void MLX90363_clearBuffers(MLX90363* dev);
uint8_t computeCRC(uint8_t* rxBuffer);
uint8_t verifyCRC(MLX90363* dev);





#endif /* INC_MLX90363_H_ */
