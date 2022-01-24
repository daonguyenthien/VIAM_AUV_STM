/*
 * User_CAN.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Chau Thanh Hai
 */
 #ifndef MY_SYSCAN_H_
#define MY_SYSCAN_H_

#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "my_bms24v40ah.h"
#include "my_bms24v10ah.h"
#include "my_altimeter.h"
#include <stdbool.h>
#include "my_io.h"
#include "my_delay.h"
#include "my_straingauge.h"

//typedef enum
//{
//	MOTOR_CW		= 'R',
//	MOTOR_CCW		= 'L',
//}USYSCAN_Direction_of_Motor_TypeDef;

typedef struct
{
	Motor_StatusTypeDef Mass_Status;
	Motor_StatusTypeDef Pistol_Status;
	UBMS40_Status_Typedef BMS40_Status;
	UBMS10_Status_Typedef BMS10_Status;
	Altimeter_StatusTypeDef ALTI_Status;
	Strain_Gauge_StatusTypeDef Status_Hull;
	uint8_t Leak_Status;
}CAN_DataTypeDef;

typedef enum
{
		//--------FUNCTION--------//
		WRITE_DATA 			= 0x00,
		READ_DATA 			= 0x01,
		STATUS_DATA 		= 0x02,
}Function_CANBUS_TypeDef;

typedef enum
{
		//--------ID BOARD ARM 1--------//
		ARM1_MASS_SHIFTER 		= 0x01,
		ARM1_PISTOL 					= 0x02,
		ARM1_LEAK_SENSOR 			= 0x03,
		ARM1_STRAIN_GAUGE 		= 0x04,
		ARM1_ALTIMETER 				= 0x05,
		ARM1_LED_DRIVER 			= 0x06,
		ARM1_ROLL_DRIVER 			= 0x07,
		ARM1_ALL_DATA 				= 0xFF,		
}ID_CANBUS_TypeDef;

typedef enum
{
		M_DESIRED 				= 0x01,
		M_ACTUAL 					= 0x02,
		M_KP							= 0x03,
		M_KI							=	0x04,
		M_KD							= 0x05,
		M_ENCODER					=	0x06,
		M_LS							= 0x07,
		M_IMOTOR					=	0x08,
		M_TEMONCHIP				=	0x09,
		M_TEMPMOTOR				=	0x0A,
		P_DESIRED 				= 0x0B,
		P_ACTUAL 					= 0x0C,
		P_KP							= 0x0D,
		P_KI							=	0x0E,
		P_KD							= 0x0F,
		P_ENCODER					=	0x10,
		P_LS							= 0x11,
		P_IMOTOR					=	0x12,
		P_TEMONCHIP				=	0x13,
		P_TEMPMOTOR				=	0x14,
		ALTIMETER_FEET		=	0x15,
		ALTIMETER_METRES	=	0x16,
		ALTIMETER_FATHOMS	=	0x17,
		LED_VREF					=	0x18,
		R_DESIRED					=	0x19,
		R_ACTUAL					=	0x1A,
		R_KP							=	0x1B,
		R_KI							=	0x1C,
		R_KD							=	0x1D,
		LEAK_POSITION			=	0xFE,
		STATUS_HULL				=	0xFF,
}REGISTER_CANBUS_TypeDef;

struct Status_flag
{
	bool Done_setup;
	
	//motor running
	bool Mass_open;
	bool Mass_Run_CW;
	bool Mass_Run_CCW;
	bool Mass_Position;
	bool Pistol_open;
	bool Pistol_Run_CW_Joystick;
	bool Pistol_Run_CCW_Joystick;
	bool Pistol_Run_CW;
	bool Pistol_Run_CCW;
	bool Pistol_Position;
	
	//data transmitting
	bool Send_Data;
	bool End_Frame_ARM2;
	bool End_Frame_Jetson;
	
	//limit switch
	bool Pis_T;
	bool Pis_H;
	bool Mass_T;
	bool Mass_H;
	bool LK_PE13;
};
extern struct Status_flag Flag;

extern float UPWM_DutyCycle_Pistol;
extern float UPWM_DutyCycle_Mass;
extern float UPWM_Position_Pistol;
extern float UPWM_Position_Mass;

extern float i1,i2;
extern float z;

void USYSCAN_Configure(void);
void USYSCAN_Respond_ALLData(CAN_DataTypeDef *_can_data);
bool USYSCAN_NeedRespondData(void);
void USYSCAN_AllowRespondData(FunctionalState NewState);
void USYSCAN_AllowRespondCheckSystem(FunctionalState NewState);
void USYSCAN_SystemReady(void);
void USYSCAN_SystemNotReady(void);
bool USYSCAN_IsSystemReady(void);
bool USYSCAN_IsNeedCheckSystem(void);
void USYSCAN_PassSystemReady(FunctionalState NewState);
void USYSCAN_Transmit(int _IDstd,int _length, uint8_t _data[]);
void USYSCAN_SystemReady_2(void);
void USYSCAN_Send_Data(void);
void USYSCAN_OpenThruster(void);
void USYSCAN_Send_test(void);
void USYSCAN_Test(void);

#endif /* USER_CAN_H_ */
