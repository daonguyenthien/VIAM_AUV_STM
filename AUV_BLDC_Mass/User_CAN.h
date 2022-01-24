/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USER_CAN_H
#define __USER_CAN_H

#ifdef __cplusplus
 extern "C" {
#endif
	
	 
/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"	 
#include <stdbool.h>
#include "User_Delay.h"
	 
/* Exported constants/macros -------------------------------------------------*/


 /**
 * @defgroup <Group name here>
 * @{
 */	
 
 /**
 * @}
 */	 
/* Exported types ------------------------------------------------------------*/
//#define Flag_Open_Loop					0
//#define Flag_PID_Control				1
//#define Flag_Fuzzy_Control			2
//#define Flag_Motor_Run_CW				3
//#define Flag_Motor_Run_CCW			4

typedef enum {Flag_Open_Loop = 0, Flag_PID_Control = 1, Flag_Fuzzy_Control = 2, Flag_Motor_Run_CW = 3, Flag_Motor_Run_CCW = 4, Flag_Motor_Position = 5}CAN_Flag;
	 
/* Exported function prototypes ----------------------------------------------*/
void UCAN_GPIO_Configure(void);
void UCAN_CAN_Configure(void);
void UCAN_Transmit(CAN_TypeDef* CANx, int _IDstd,int _length, uint8_t _data[]);
uint8_t UCAN_GetMessage(uint8_t bytes);
void UCAN_SetFlag(CAN_Flag type);
bool UCAN_GetFlag(CAN_Flag type);
void UCAN_ResetFlag(void);
uint8_t UCAN_Checksum(uint8_t *_data);
void UCAN_SystemOverLoad(void);
void UCAN_StickedMotor(void);
void UCAN_Respond_ALLData(void);
bool UCAN_NeedRespondData(void);
void UCAN_SystemReady(void);
void UCAN_SystemNotReady(void);
void UCAN_SendPosition(void);
bool UCAN_IsSystemReady(void);
void UCAN_PassSystemReady(FunctionalState NewState);
void UCAN_AllowRespondData(FunctionalState NewState);
void UCAN_AllowRespondCheckSystem(FunctionalState NewState);
bool UCAN_IsNeedCheckSystem(void);
void UCAN_Test(void);
void UCAN_Test_2(void);

void UCAN_Thruster(void);
void UCAN_Rudder(void);
void UCAN_Mass(void);
void UCAN_Pistol(void);
void UCAN_Altimeter(void);
void UCAN_Leaksensor(void);

extern bool flag_run;
/* Peripherals Interrupt prototypes ------------------------------------------*/


#ifdef __cplusplus
}
#endif

#endif 
