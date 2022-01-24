/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_MX28_H
#define __MY_MX28_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "stm32f4xx.h"	 
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_usart.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_dma.h" 
#include "misc.h"
#include "my_delay.h"
	 
/* Exported constants/macros -------------------------------------------------*/
//Reg Control
#define INT_PING			0x01
#define INT_READ			0x02
#define INT_WRITE			0x03
#define INT_REGWRITE		0x04
#define INT_ACTION			0x05
#define INT_FACTORYRESET	0x06
#define INT_REBOOT			0x08
#define INT_SYNCWRITE		0x83
#define INT_BULKREAD		0x92

/* Exported types ------------------------------------------------------------*/

typedef enum 
{
	//Baudrate
	BPS_1000000			= 1,
	BPS_500000			= 3,
	BPS_400000			= 4,
	BPS_250000			= 7,
	BPS_200000			= 9,
	BPS_115200			= 16,
	BPS_57600			= 34,
	BPS_19200			= 103,
	BPS_9600			= 207,
}MX28_Baudrate_Typedef;

typedef enum 
{
	MODEL_NUMBER_L			= 0x00,
	MODEL_NUMBER_H			= 0x01,
	VERSION_OF_FIRMWARE		= 0x02,
	ID						= 0x03,
	BAUDRATE				= 0x04,
	RETURN_DELAY_TIME		= 0x05,
	CW_ANGLE_LIMIT_L		= 0x06,
	CW_ANGLE_LIMIT_H		= 0x07,
	CCW_ANGLE_LIMIT_L		= 0x08,
	CCW_ANGLE_LIMIT_H		= 0x09,
	THE_HIGHEST_LIMIT_TEMP  = 0x0B,
	THE_LOWEST_LIMIT_VOL	= 0x0C,
	THE_HIGHEST_LIMIT_VOL	= 0x0D,
	MAX_TORQUE_L			= 0x0E,
	MAX_TORQUE_H			= 0x0F,
	STATUS_RETURN_LEVEL		= 0x10,
	ALARM_LED				= 0x11,
	ALARM_SHUTDOWN			= 0x12,
	MULTI_TURN_OFFSET_L		= 0x14,
	MULTI_TURN_OFFSET_H		= 0x15,
	RESOLUTION_DIVIDER		= 0x16,
	TORQUE_ENABLE			= 0x18,
	LED						= 0x19,
	D_GAIN					= 0x1A,
	I_GAIN					= 0x1B,
	P_GAIN					= 0x1C,
	GOAL_POSITION_L			= 0x1E,
	GOAL_POSITION_H			= 0x1F,
	MOVING_SPEED_L			= 0x20,
	MOVING_SPEED_H			= 0x21,
	TORQUE_LIMIT_L			= 0x22,
	TORQUE_LIMIT_H			= 0x23,
	PRESENT_POSITION_L		= 0x24,
	PRESENT_POSITION_H		= 0x25,
	PRESENT_SPEED_L			= 0x26,
	PRESENT_SPEED_H			= 0x27,
	PRESENT_LOAD_L			= 0x28,
	PRESENT_LOAD_H			= 0x29,
	PRESENT_VOLTAGE			= 0x2A,
	PRESENT_TEMP			= 0x2B,
	MREGISTERED				= 0x2C,
	MOVING					= 0x2E,
	LOCK					= 0x2F,
	PUNCH_L					= 0x30,
	PUNCH_H					= 0x31,
	GOAL_ACC				= 0x49,
}MX28_Address_Typedef;

typedef struct
{
	float Position;
	float Speed;
	float Load;
	float Voltage;
	float Temperature;
}MX28_Status_Typedef;

/* Exported function prototypes ----------------------------------------------*/
void UMX28_Configuration(void);
void UMX28_Init(void);

void UMX28_readStatusData(uint8_t _id);
void UMX28_readVersionData(uint8_t _id);
void UMX28_setGoalPosition(uint8_t _id, uint16_t _data);
void UMX28_setMovingSpeed(uint8_t _id, uint16_t _data);
void UMX28_setKp(uint8_t _id, uint8_t _data);
void UMX28_setKi(uint8_t _id, uint8_t _data);
void UMX28_setKd(uint8_t _id, uint8_t _data);
void UMX28_pingServo(uint8_t _id);
void UMX28_getPIDGain(uint8_t _id);
void UMX28_setBaudrate(uint8_t _id, MX28_Baudrate_Typedef _baudrate);
float UMX28_getStallTorque(float _voltage);
void UMX28_getStatus(MX28_Status_Typedef *_mx28_status);

/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
