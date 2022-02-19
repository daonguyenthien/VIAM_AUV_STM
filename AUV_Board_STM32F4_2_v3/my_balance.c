#include "my_balance.h"

/**
 * @defgroup Module Pin define
 * @{
 */	
			/** 
			* @brief   Timer Module define 
			*/
			#define 	ALGORITHM_TIM  				      				TIM9
			#define 	ALGORITHM_TIM_CLK					 	 				RCC_APB2Periph_TIM9
			#define 	ALGORITHM_TIM_CLK_Cmd    						RCC_APB2PeriphClockCmd
			#define 	ALGORITHM_TIM_IRQn    							TIM1_BRK_TIM9_IRQn				
			#define		ALGORITHM_TIM_PreemptionPriority		0x02
			#define		ALGORITHM_TIM_SubPriority						0x02		
			
			#define 	ALGORITHM_TIM_IRQHandler						TIM1_BRK_TIM9_IRQHandler	

#define SAMPLE_TIME 20 // (ms)
#define pi 3.14159
#define THETA_MIN -pi/3
#define THETA_MAX pi/3
#define K1 3/pi
#define THETADOT_MIN -1	//(rad/s)
#define THETADOT_MAX 1
#define K2 1
#define Ku 70

union Data_In theta_in, velocity_out;
float pre_theta_in, thetadot_in;
float velocity_defuzzy;
double theta_filtered, theta_dot_filetered;
static bool first_data_pass = true;

struct
{
	double NB;
	double NS;
	double ZE;
	double PB;
	double PS;
}muy_theta, muy_thetadot;

struct
{
	double NVB;
	double NB;
	double NM;
	double NS;
	double ZE;
	double PS;
	double PM;
	double PB;
	double PVB;
}velocity, muy_velocity, sum;

struct
{
	double C1;
	double C2;
}theta, thetadot;

void Algorithm_Config(void)
{
	TIM_TimeBaseInitTypeDef TIM_InitStructure;
	NVIC_InitTypeDef	NVIC_InitStruct;
	
	ALGORITHM_TIM_CLK_Cmd(ALGORITHM_TIM_CLK,ENABLE);
	TIM_InitStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_InitStructure.TIM_Prescaler = 72*100-1;
	TIM_InitStructure.TIM_Period = 300;
	TIM_TimeBaseInit(ALGORITHM_TIM, &TIM_InitStructure);
	TIM_Cmd(ALGORITHM_TIM, ENABLE);
	TIM_ITConfig(ALGORITHM_TIM, TIM_IT_Update, ENABLE);
	
	NVIC_InitStruct.NVIC_IRQChannel = ALGORITHM_TIM_IRQn;
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = ALGORITHM_TIM_PreemptionPriority;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = ALGORITHM_TIM_SubPriority;
	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void Membership_Function_Declaration(void)
{
	//theta
	theta.C1 = 0.4;
	theta.C2 = 0.8;
	
	//thetadot
	thetadot.C1 = 0.4;
	thetadot.C2 = 0.8;
	
	//velocity
	velocity.PB = 1;
	velocity.PM = 0.8;
	velocity.PS = 0.4;
	velocity.ZE = 0;
	velocity.NS = -velocity.PS;
	velocity.NM = -velocity.PM;
	velocity.NB = -velocity.PB;
}

float x2xdot(float x, float pre_x)
{
	return (x - pre_x)/(SAMPLE_TIME*0.001);
}

double Fuzzy_Trapzoid(double x, double L, double C1, double C2, double R)
{
	if(x <= L)
	{
		return 0;
	}
	else if(x > L && x < C1)
	{
		return (x - L)/(C1 - L);
	}
	else if(x >= C1 && x <= C2)
	{
		return 1;
	}
	else if(x > C2 && x < R)
	{
		return (R - x)/(R - C2);
	}
	else if(x >= R)
	{
		return 0;
	}
}

void Fuzzy_Result(double x1, double x2)
{
	muy_theta.NB = Fuzzy_Trapzoid(x1,-2,-1,-theta.C2,-theta.C1);
	muy_theta.NS = Fuzzy_Trapzoid(x1,-theta.C2,-theta.C1,-theta.C1,0);
	muy_theta.ZE = Fuzzy_Trapzoid(x1,-theta.C1,0,0,theta.C1);
	muy_theta.PS = Fuzzy_Trapzoid(x1,0,theta.C1,theta.C1,theta.C2);
	muy_theta.PB = Fuzzy_Trapzoid(x1,theta.C1,theta.C2,1,2);
	
	muy_thetadot.NB = Fuzzy_Trapzoid(x2,-2,-1,-theta.C2,-theta.C1);
	muy_thetadot.NS = Fuzzy_Trapzoid(x2,-theta.C2,-theta.C1,-theta.C1,0);
	muy_thetadot.ZE = Fuzzy_Trapzoid(x2,-theta.C1,0,0,theta.C1);
	muy_thetadot.PS = Fuzzy_Trapzoid(x2,0,theta.C1,theta.C1,theta.C2);
	muy_thetadot.PB = Fuzzy_Trapzoid(x2,theta.C1,theta.C2,1,2);
}

static double min(double x, double y)
{
	if(x > y)
		return y;
	else
		return x;
}

static float map(uint16_t x, uint16_t in_min, uint16_t in_max, float out_min, float out_max) 
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}	

void Control_Laws(Defuzzy_And_Or_Method method, double x1, double x2,int8_t y)
{
	double temp = 0;
	if(method == MAX_MIN)
	{
		temp = min(x1,x2);
	}
	else if(method == MAX_PROD)
	{
		temp = x1*x2;
	}
	switch(y)
	{
		case NVB:
			if(temp > muy_velocity.NVB)
				muy_velocity.NVB = temp;
			sum.NVB += temp;
		break;
		case NB:
			if(temp > muy_velocity.NB)
				muy_velocity.NB = temp;
			sum.NB += temp;
		break;
		case NM:
			if(temp > muy_velocity.NM)
				muy_velocity.NM = temp;
			sum.NM += temp;
		break;
		case NS:
			if(temp > muy_velocity.NS)
				muy_velocity.NS = temp;
			sum.NS += temp;
		break;
		case ZE:
			if(temp > muy_velocity.ZE)
				muy_velocity.ZE = temp;
			sum.ZE += temp;
		break;
		case PS:
			if(temp > muy_velocity.PS)
				muy_velocity.PS = temp;
			sum.PS += temp;
		break;
		case PM:
			if(temp > muy_velocity.PM)
				muy_velocity.PM = temp;
			sum.PM += temp;
		break;
		case PB:
			if(temp > muy_velocity.PB)
				muy_velocity.PB = temp;
			sum.PB += temp;
		break;
		case PVB:
			if(temp > muy_velocity.PVB)
				muy_velocity.PVB = temp;
			sum.PVB += temp;
		break;
	}
}

float Defuzzy_Result(Defuzzy_And_Or_Method method)
{
	Control_Laws(method,muy_theta.NB,muy_thetadot.NB,PVB);
	Control_Laws(method,muy_theta.NB,muy_thetadot.NS,PB);
	Control_Laws(method,muy_theta.NB,muy_thetadot.ZE,PM);
	Control_Laws(method,muy_theta.NB,muy_thetadot.PS,PS);
	Control_Laws(method,muy_theta.NB,muy_thetadot.PB,ZE);
	
	Control_Laws(method,muy_theta.NS,muy_thetadot.NB,PB);
	Control_Laws(method,muy_theta.NS,muy_thetadot.NS,PM);
	Control_Laws(method,muy_theta.NS,muy_thetadot.ZE,PS);
	Control_Laws(method,muy_theta.NS,muy_thetadot.PS,ZE);
	Control_Laws(method,muy_theta.NS,muy_thetadot.PB,NS);
	
	Control_Laws(method,muy_theta.ZE,muy_thetadot.NB,PM);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.NS,PS);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.ZE,ZE);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.PS,NS);
	Control_Laws(method,muy_theta.ZE,muy_thetadot.PB,NM);
	
	Control_Laws(method,muy_theta.PS,muy_thetadot.NB,PS);
	Control_Laws(method,muy_theta.PS,muy_thetadot.NS,ZE);
	Control_Laws(method,muy_theta.PS,muy_thetadot.ZE,NS);
	Control_Laws(method,muy_theta.PS,muy_thetadot.PS,NM);
	Control_Laws(method,muy_theta.PS,muy_thetadot.PB,NB);
	
	Control_Laws(method,muy_theta.PB,muy_thetadot.NB,ZE);
	Control_Laws(method,muy_theta.PB,muy_thetadot.NS,NS);
	Control_Laws(method,muy_theta.PB,muy_thetadot.ZE,NM);
	Control_Laws(method,muy_theta.PB,muy_thetadot.PS,NB);
	Control_Laws(method,muy_theta.PB,muy_thetadot.PB,NVB);
	
	double TS = sum.NVB*velocity.NVB + sum.PVB*velocity.PVB +
							sum.NB*velocity.NB + sum.PB*velocity.PB +
							sum.NM*velocity.NM + sum.PM*velocity.PM +
							sum.NS*velocity.NS + sum.PS*velocity.PS +
							sum.ZE*velocity.ZE;
	double MS = sum.NVB + sum.NB + sum.NM + sum.NS + sum.ZE +
							sum.PVB + sum.PB + sum.PM + sum.PS;
	return (float)(TS/MS);
}

void Filter_In(float x1, float x2)
{
	if(x1 < THETA_MIN)
	{
		x1 = THETA_MIN;
	}
	else if(x1 > THETA_MAX)
	{
		x1 = THETA_MAX;
	}
	if(x2 < THETADOT_MIN)
	{
		x2 = THETADOT_MIN;
	}
	else if(x2 > THETADOT_MAX)
	{
		x2 = THETADOT_MAX;
	}
	theta_filtered = (double)x1*K1;
	theta_dot_filetered = (double)x2*K2;
}

void Reset_Value(void)
{
	muy_velocity.NVB = 0;
	muy_velocity.NB = 0;
	muy_velocity.NM = 0;
	muy_velocity.NS = 0;
	muy_velocity.ZE = 0;
	muy_velocity.PS = 0;
	muy_velocity.PM = 0;
	muy_velocity.PB = 0;
	muy_velocity.PVB = 0;
	
	sum.NVB = 0;
	sum.NB = 0;
	sum.NM = 0;
	sum.NS = 0;
	sum.ZE = 0;
	sum.PS = 0;
	sum.PM = 0;
	sum.PB = 0;
	sum.PVB = 0;
}

void Balance_Operation(void)
{
	theta_in.value = map(xsen_angle,0,4095,-pi/3,pi/3);
	thetadot_in = x2xdot(theta_in.value, pre_theta_in);
	Filter_In(theta_in.value, thetadot_in);
	//Fuzzy Logic Controller
	//************************************************
	Fuzzy_Result(theta_filtered, theta_dot_filetered);
	velocity_defuzzy = Defuzzy_Result(MAX_MIN);
	//************************************************
	velocity_out.value = velocity_defuzzy*Ku;
	if(first_data_pass)
		first_data_pass = false;
	else
		UCAN_Send_Mass_Speed(velocity_out.value);
	pre_theta_in = theta_in.value;
	Reset_Value();
	
}

void ALGORITHM_TIM_IRQHandler(void)
{
	if(TIM_GetITStatus(ALGORITHM_TIM, TIM_IT_Update) != RESET)
	{
		//Balance_Operation();
		TIM_ClearITPendingBit(ALGORITHM_TIM, TIM_IT_Update);
	}
}
