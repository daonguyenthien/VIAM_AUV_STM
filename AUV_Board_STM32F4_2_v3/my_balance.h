#ifndef _MY_BALANCE_H_
#define _MY_BALANCE_H_

#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include <stdbool.h>
#include "my_mx28.h"
#include "my_bms24v40ah.h"
#include "my_keller_pa3.h"
#include "my_io.h"
#include "my_can.h"
#include "my_delay.h"
#include "my_adc.h"

typedef enum
{
	NVB = -4,
	NB = -3,
	NM = -2,
	NS = -1,
	ZE = 0,
	PS = 1,
	PM = 2,
	PB = 3,
	PVB = 4
}Fuzzy_DataTypeDef;

typedef enum
{
	MAX_MIN = 0,
	MAX_PROD = 1
}Defuzzy_And_Or_Method;

//PP giai mo: trung binh co trong so

union Data_In
{
	float value;
	struct
	{
		uint8_t byte1;
		uint8_t byte2;
		uint8_t byte3;
		uint8_t byte4;
	}byte;
};
extern union Data_In theta_in, velocity_out;

void Algorithm_Config(void);
void Membership_Function_Declaration(void);
float x2xdot(float x, float pre_x);
double Fuzzy_Trapzoid(double x, double L, double C1, double C2, double R);
void Filter_In(float x1, float x2);
void Fuzzy_Result(double x1, double x2);
void Control_Laws(Defuzzy_And_Or_Method method, double x1, double x2,int8_t y);
float Defuzzy_Result(Defuzzy_And_Or_Method method);
void Reset_Value(void);
void Balance_Operation(void);

#endif
