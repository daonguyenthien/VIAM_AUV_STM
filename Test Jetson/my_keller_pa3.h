/*
 * my_keller_pa3.h
 *
 *  Created on: Apr 11, 2018
 *      Author: Chau Thanh Hai
 */
#include "misc.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "my_delay.h"
#include "stm32f4xx_dma.h"
#include <stdbool.h>

#ifndef USER_SERIAL_H_
#define USER_SERIAL_H_

#define MODBUS_READ					0x03	
#define MODBUS_WRITE				0x06
#define MODBUS_ECHO					0x08
#define MODBUS_WRITEMULTI		0x10
#define KELLER_READ					0x49	//function 73
#define KELLER_INIT					0x30	//function 48

typedef struct
{
	float Pressure;
	float Temperature;
}UKellerPA3_Status_Typedef;

void UKELLER_Configuration(void);
void UKELLER_MODBUS_Read(uint8_t DevAddr, uint16_t StAddr, uint16_t NoReg);
void UKELLER_KELLER_ReadP1(uint8_t DevAddr);
void UKELLER_KELLER_ReadTOB1(uint8_t DevAddr);
void UKELLER_KELLER_Init(uint8_t DevAddr);
void UKELLER_MODBUS_WriteSingle(uint8_t DevAddr, uint16_t StAddr, uint16_t Data);
void UKELLER_TxData(uint8_t data);
void UKELLER_GetStatus(UKellerPA3_Status_Typedef *_ukeller_status);
void CalcCRC16(uint8_t _buffer[], uint8_t _length, unsigned char* CRC_H, unsigned char* CRC_L);

#endif /* USER_SERIAL_H_ */
