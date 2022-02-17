/*
 * User_CAN.h
 *
 *  Created on: Apr 9, 2018
 *      Author: Chau Thanh Hai
 */
#ifndef MY_PMCAN_H_
#define MY_PMCAN_H_

#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "my_bms24v40ah.h"
#include "my_bms24v10ah.h"
#include "my_altimeter.h"
#include <stdbool.h>
#include "my_delay.h"
#include "my_straingauge.h"

typedef struct
{
	float DesiredSpeed;
	float ActualSpeed;
	float Encoder;
	float LimitSwitch;
	float iMotor;
	float TempOnChip;
	float TempAmbient;
}Motor_StatusTypeDef;

typedef enum
{
	MOTOR_CW		= 'R',
	MOTOR_CCW		= 'L',
}UPMCAN_Direction_of_Motor_TypeDef;

typedef enum
{
	DISABLE_DRIVER	= 'C',
	ENABLE_DRIVER		= 'O',
}UPMCAN_ENABLE_DRIVER_TypeDef;

extern float Pistol_Actual_Position;
extern float Mass_Actual_Position;

void UPMCAN_Configure(void);
void UPMCAN_Initialize_Position(void);
void UPMCAN_Send_Position_Data(void);

void UPMCAN_Test_Pistol(void);
//-------Function For Mass Shifter-------//
void UPMCAN_Mass_Start(UPMCAN_ENABLE_DRIVER_TypeDef _state);
void UPMCAN_Mass_SetOLoop_Duty(float _duty, UPMCAN_Direction_of_Motor_TypeDef _direction);
void UPMCAN_Mass_SetPID_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction);
void UPMCAN_Mass_SetPID_Kp(float _kp);
void UPMCAN_Mass_SetPID_Ki(float _ki);
void UPMCAN_Mass_SetPID_Kd(float _kd);
void UPMCAN_Mass_SetFuzzy_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction);
void UPMCAN_Mass_SetFuzzy_Ge(float _ge);
void UPMCAN_Mass_SetFuzzy_Gde(float _gde);
void UPMCAN_Mass_SetFuzzy_Gdu(float _gdu);
void UPMCAN_Mass_GetStatus(Motor_StatusTypeDef _motor_status);
void UPMCAN_Mass_Position(float position);

//-------Function For Pistol-------//
void UPMCAN_Pistol_Start(UPMCAN_ENABLE_DRIVER_TypeDef _state);
void UPMCAN_Pistol_SetOLoop_Duty(float _duty, UPMCAN_Direction_of_Motor_TypeDef _direction);
void UPMCAN_Pistol_SetPID_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction);
void UPMCAN_Pistol_SetPID_Kp(float _kp);
void UPMCAN_Pistol_SetPID_Ki(float _ki);
void UPMCAN_Pistol_SetPID_Kd(float _kd);
void UPMCAN_Pistol_SetFuzzy_DesSpeed(float _speed, UPMCAN_Direction_of_Motor_TypeDef _direction);
void UPMCAN_Pistol_SetFuzzy_Ge(float _ge);
void UPMCAN_Pistol_SetFuzzy_Gde(float _gde);
void UPMCAN_Pistol_SetFuzzy_Gdu(float _gdu);
void UPMCAN_Pistol_GetStatus(Motor_StatusTypeDef _motor_status);

#endif /* USER_CAN_H_ */
