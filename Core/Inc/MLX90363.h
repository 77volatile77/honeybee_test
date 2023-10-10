/*
 * MLX90363.h
 *
 *  Created on: Aug 19, 2023
 *      Author: uzairsyed
 */

#ifndef INC_MLX90363_H_
#define INC_MLX90363_H_

#include "stm32l4xx.h"
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
	Alpha 		= 0, // We clear the entire byte, so we don't have to use &= ~(3 << 6).
	Alpha_Beta 	= (1 << 6),
	XYZ 		= (2 << 6)
}MessageType;


/******************* EEPROM MESSAGES *****************/
typedef enum {

}EEWriteMessages;


typedef struct{
	SPI_HandleTypeDef* spi_h;
	uint8_t txBuffer[8];
	uint8_t rxBuffer[8];
	uint64_t deviceID;
}MLX90363;

/**
 * INITIALIZATION
 * Returns an error code: 0xBAD if something went wrong.
 * Consider adding other parameters (idk yet).
 */
uint8_t MLX90363_init(SPI_HandleTypeDef* spi_h, MLX90363* dev);

#endif /* INC_MLX90363_H_ */
