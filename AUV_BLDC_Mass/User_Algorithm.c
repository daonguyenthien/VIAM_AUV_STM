/* Includes ------------------------------------------------------------------*/
#include "User_Algorithm.h"
#include "User_HALLSensor.h"
#include "User_Encoder.h"
#include "User_PWM.h"
#include "User_CAN.h"
#include "User_Led.h"
/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/
static union Data_Algorithm
{
	float Value;
	
	struct
	{
		unsigned a1:8;
		unsigned a2:8;
		unsigned a3:8;
		unsigned a4:8;
	} byte;
} UpdateParameters;

static struct
{
	float Kp;
	float Ki;
	float Kd;
}	PID_Parameter;

static struct
{
	float Ge;
	float Gde;
	float Gdu;
} Fuzzy_Parameter;

static struct 
{
	float P;
	float I;
	float D;
}	PID_part;

//yk = uk + a*uk1 + b*yk1 
static struct{
	float a;
	float b; 
} LowPassFilter_Parameter;

static float UALTHM_SetSpeed = 0, SpeedError = 0, Pre_SpeedError = 0, Pre_Pre_SpeedError = 0, _SetSpeedFiltered = 0;
static float UALTHM_SetPosition = 0.0f;
int32_t _Position_Pulse = 0;
int32_t PositionError = 0, Pre_PositionError = 0, Pre_Pre_PositionError = 0;
static uint32_t _uk = 0, _uk1 = 0, _yk = 0, _yk1 = 0;

int _DutyCycle = 0;
/* Private const/macros ------------------------------------------------------*/
#define DU_NB 	-3
#define DU_NM 	-2
#define DU_NS 	-1
#define DU_ZE 	 0
#define DU_PS 	 1
#define DU_PM 	 2
#define DU_PB 	 3
#define min(x,y) ((x<y)?(x):(y))


/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
uint32_t UALTHM_SetPointFilter(uint32_t setPoint);
/* Exported function body ----------------------------------------------------*/

/* Private functions body ----------------------------------------------------*/
void UALTHM_UpdateParameters(UPDATE_TYPE type, uint8_t a1, uint8_t a2, uint8_t a3, uint8_t a4)
{
	UpdateParameters.byte.a1 = a1;
	UpdateParameters.byte.a2 = a2;
	UpdateParameters.byte.a3 = a3;
	UpdateParameters.byte.a4 = a4;
	
	switch(type)
	{
		case Update_Kp:
		{
			PID_Parameter.Kp = UpdateParameters.Value;
			break;
		}
		case Update_Ki:
		{
			PID_Parameter.Ki = UpdateParameters.Value;
			break;
		}
		case Update_Kd:
		{
			PID_Parameter.Kd = UpdateParameters.Value;
			break;
		}
		case Update_Ge:
		{
			Fuzzy_Parameter.Ge = UpdateParameters.Value;
			break;
		}
		case Update_Gde:
		{
			Fuzzy_Parameter.Gde = UpdateParameters.Value;
			break;
		}
		case Update_Gdu:
		{
			Fuzzy_Parameter.Gdu = UpdateParameters.Value;
			break;
		}
		case Update_SetSpeed:
		{
			UALTHM_SetSpeed = (int)UpdateParameters.Value;
			break;
		}
		case Update_SetPosition:
		{
			UALTHM_SetPosition = (int)UpdateParameters.Value;
		}
	}
}

void UALTHM_UpdateSetPosition(float position){
	UALTHM_SetPosition = position;
}

void UALTHM_UpdatePIDParams(float _kp, float _ki, float _kd)
{
	PID_Parameter.Kp = _kp;
	PID_Parameter.Ki = _ki;
	PID_Parameter.Kd = _kd;
}

void UALTHM_UpdateFuzzyParams(float _ge, float _gde, float _gdu)
{
	Fuzzy_Parameter.Ge = _ge;
	Fuzzy_Parameter.Gde = _gde;
	Fuzzy_Parameter.Gdu = _gdu;
}

uint8_t UALTHM_GetBytesSetSpeed(uint8_t byte)
{
	UpdateParameters.Value = (float)UALTHM_SetSpeed;
	uint8_t value = 0;
	if(byte == 1) value = UpdateParameters.byte.a1;
	else if( byte == 2) value = UpdateParameters.byte.a2;
	else if( byte == 3) value = UpdateParameters.byte.a3;
	else if (byte == 4) value = UpdateParameters.byte.a4;
	return value;
}

uint8_t UALTHM_GetBytesSetPosition(uint8_t byte)
{
	UpdateParameters.Value = (float)UALTHM_SetPosition;
	uint8_t value = 0;
	if(byte == 1) value = UpdateParameters.byte.a1;
	else if( byte == 2) value = UpdateParameters.byte.a2;
	else if( byte == 3) value = UpdateParameters.byte.a3;
	else if (byte == 4) value = UpdateParameters.byte.a4;
	return value;
}

//void UALTHM_PID_v1(bool _state_Motor)
//{
//	SpeedError = UALTHM_SetSpeed - UHALL_GetActualSpeed();
//	PID_part.P = PID_Parameter.Kp*SpeedError;
//	PID_part.I += PID_Parameter.Ki*SpeedError*((float)UALTHM_TIME_SAMPLING/1000.0);
//	PID_part.D = PID_Parameter.Kd*(SpeedError - Pre_SpeedError)/((float)UALTHM_TIME_SAMPLING/1000.0);
//	
//	if(PID_part.I < -(float)PWM_DC_MAX/10.0) PID_part.I = -(float)PWM_DC_MAX/10.0;
//	else if(PID_part.I > (float)PWM_DC_MAX/10.0) PID_part.I = (float)PWM_DC_MAX/10.0;
//	
//	_DutyCycle += (int)(PID_part.P + PID_part.I + PID_part.D);
//	Pre_SpeedError = SpeedError;
//	
//	if(_DutyCycle < PWM_DC_MIN)
//	{
//		_DutyCycle = PWM_DC_MIN;
//		PID_part.I = 0;
//	}
//	else if(_DutyCycle > PWM_DC_MAX)
//	{
//		_DutyCycle = PWM_DC_MAX;
//		PID_part.I = 0;
//	}
//	
//	UHALL_ReadPosition(_state_Motor);
//	UPWM_SetDutyCycle(_DutyCycle);
//}

void UALTHM_PID_v2(bool _state_Motor)
{	
//	float Time_Sampling = (float)UALTHM_TIME_SAMPLING/1000.0;
//	SpeedError = UALTHM_SetSpeed - UHALL_GetActualSpeed();
//	
//	PID_part.P = 2*Time_Sampling*PID_Parameter.Kp + PID_Parameter.Ki*Time_Sampling*Time_Sampling + 2*PID_Parameter.Kd;
//	PID_part.I = PID_Parameter.Ki*Time_Sampling*Time_Sampling - 2*Time_Sampling*PID_Parameter.Kp - 4*PID_Parameter.Kd;
//	PID_part.D = 2*PID_Parameter.Kd;
//	
//	_DutyCycle += (int)((PID_part.P*SpeedError + PID_part.I*Pre_SpeedError + PID_part.D*Pre_Pre_SpeedError)/(2*Time_Sampling));
//	
//	Pre_Pre_SpeedError = Pre_SpeedError;
//	Pre_SpeedError = SpeedError;
//	
//	if(_DutyCycle < PWM_DC_MIN)
//	{
//		_DutyCycle = PWM_DC_MIN;
//	}
//	else if(_DutyCycle > PWM_DC_MAX)
//	{
//		_DutyCycle = PWM_DC_MAX;
//	}
//		
//	UHALL_ReadPosition(_state_Motor);
//	UPWM_SetDutyCycle(_DutyCycle);
	static int _preDutyCycle = 0;
	_SetSpeedFiltered = UALTHM_SetPointFilter(UALTHM_SetSpeed);
	if (UALTHM_SetSpeed - _SetSpeedFiltered < 10) _SetSpeedFiltered = UALTHM_SetSpeed;
	SpeedError = _SetSpeedFiltered - UENC_GetSpeed();
	PID_part.P = PID_Parameter.Kp * (SpeedError - Pre_SpeedError);
 	PID_part.I = PID_Parameter.Ki * (SpeedError + Pre_SpeedError) * UALTHM_TIME_SAMPLING/2000;
	PID_part.D = PID_Parameter.Kd * (SpeedError - 2*Pre_SpeedError + Pre_Pre_SpeedError)*1000/UALTHM_TIME_SAMPLING;
	_DutyCycle += _preDutyCycle + PID_part.P + PID_part.I + PID_part.D;
	Pre_Pre_SpeedError = Pre_SpeedError;
	Pre_SpeedError = SpeedError;
	if (_DutyCycle > PWM_DC_MAX) _DutyCycle = PWM_DC_MAX;
	if (_DutyCycle < PWM_DC_MIN) _DutyCycle = PWM_DC_MIN;
	UHALL_ReadPosition(_state_Motor);
	UPWM_SetDutyCycle(_DutyCycle);
}

void UALTHM_PID_v3(void){
//	static int32_t _PrePositionPulse=0;
//	int32_t Position_Pulse = UENCmm2pulse(UALTHM_SetPosition);
//	int32_t _Position_Pulse = UALTHM_SetPointFilter(Position_Pulse);
//	PositionError = _Position_Pulse - UENC_GetCounterValue();
//	if (PositionError <0) PositionError = -PositionError;
//	if ((PositionError> DEADBAND) || (PositionError<-DEADBAND)){
//		PID_part.P = PID_Parameter.Kp*PositionError;
//		PID_part.I += PID_Parameter.Ki*PositionError*((float)UALTHM_TIME_SAMPLING/1000.0);
//		PID_part.D = PID_Parameter.Kd*(PositionError - Pre_PositionError)/((float)UALTHM_TIME_SAMPLING/1000.0);	
//		if(PID_part.I < -(float)PWM_DC_MAX/10.0) PID_part.I = -(float)PWM_DC_MAX/10.0;
//		else if(PID_part.I > (float)PWM_DC_MAX/10.0) PID_part.I = (float)PWM_DC_MAX/10.0;
//		
//		_DutyCycle += (int)(PID_part.P + PID_part.I + PID_part.D);
//		Pre_PositionError = PositionError;
//		if(_DutyCycle < PWM_DC_MIN)
//		{
//			_DutyCycle = PWM_DC_MIN;
//			PID_part.I = 0;
//		}
//		else if(_DutyCycle > PWM_DC_MAX)
//		{
//			_DutyCycle = PWM_DC_MAX;
//			PID_part.I = 0;
//		}
//		
//		if (_Position_Pulse - _PrePositionPulse>0) {
//			UCAN_SetFlag(Flag_Motor_Run_CW);
//			UHALL_ReadPosition(Motor_CW);
//			
//		}
//		else if (_Position_Pulse - _PrePositionPulse<0){
//			UCAN_SetFlag(Flag_Motor_Run_CCW);
//			UHALL_ReadPosition(Motor_CCW);
//		}
//		UPWM_SetDutyCycle(_DutyCycle);
//	}
//	else {
//		UPWM_SetDutyCycle(0);
//		UPWM_StopPWM();	
//	}
//	
//	_PrePositionPulse = _Position_Pulse;
	static int32_t _PrePositionPulse=0;
	static int32_t _DutyCycle_Temp;
	int32_t Position_Pulse = UENCmm2pulse(UALTHM_SetPosition);
  _Position_Pulse = UALTHM_SetPointFilter(Position_Pulse);
	PositionError = _Position_Pulse - UENC_GetCounterValue();
	//static int32_t preDutyCycle = 0;
	//if (PositionError <0) PositionError = -PositionError;
	if ((PositionError> DEADBAND) || (PositionError<-DEADBAND)){
		PID_part.P = PID_Parameter.Kp*(PositionError - Pre_PositionError);
		PID_part.I = PID_Parameter.Ki*UALTHM_TIME_SAMPLING/2000 * (PositionError + Pre_PositionError);
		PID_part.D = PID_Parameter.Kd*1000/UALTHM_TIME_SAMPLING*(PositionError - 2*Pre_PositionError + Pre_Pre_PositionError);
		_DutyCycle += PID_part.P + PID_part.I + PID_part.D;
		if (_DutyCycle < -PWM_DC_MAX) _DutyCycle = -PWM_DC_MAX;
		if (_DutyCycle > PWM_DC_MAX) _DutyCycle = PWM_DC_MAX;

//		if (_DutyCycle<0) {
//			_DutyCycle_Temp = _DutyCycle;
//			_DutyCycle = - _DutyCycle;
//		}
//		if (_DutyCycle>PWM_DC_MAX) _DutyCycle = PWM_DC_MAX;
//		if (_DutyCycle<PWM_DC_MIN) _DutyCycle = PWM_DC_MIN;
		
		//preDutyCycle = _DutyCycle_Temp;
		Pre_Pre_PositionError = Pre_PositionError;
		Pre_PositionError = PositionError;
//		if ((_Position_Pulse - _PrePositionPulse) > 0 || (PositionError>0)){
//			UCAN_SetFlag(Flag_Motor_Run_CW);
//			UHALL_ReadPosition(Motor_CW);
//		}
//		else if (_Position_Pulse - _PrePositionPulse < 0 || (PositionError<0)){
//			UCAN_SetFlag(Flag_Motor_Run_CCW);
//			UHALL_ReadPosition(Motor_CCW);
//		}
		if (_DutyCycle>PWM_DC_MIN){
			UCAN_SetFlag(Flag_Motor_Run_CW);
			UHALL_ReadPosition(Motor_CW);
			UPWM_SetDutyCycle(_DutyCycle);
		}
		else if (_DutyCycle<-PWM_DC_MIN){
			UCAN_SetFlag(Flag_Motor_Run_CCW);
			UHALL_ReadPosition(Motor_CCW);
			UPWM_SetDutyCycle(-_DutyCycle);
		}
		else {
			_DutyCycle = 0;
			UPWM_SetDutyCycle(0);
			UPWM_StopPWM();
		}
	}
	else {
		_DutyCycle = 0;
		UPWM_SetDutyCycle(0);
		UPWM_StopPWM();
	}
	_PrePositionPulse = _Position_Pulse;
	
}
void UALTHM_FUZZY(bool _state_Motor)
{
	  int ChangeSpeedError;
    int i;
    int x1,x2;
    int x1_NB,x1_NS,x1_ZE,x1_PS,x1_PB;
    int x2_NE,x2_ZE,x2_PO;
    
    int fuzzy_value[15];
    int du_value[15] = {DU_NB,DU_NM,DU_NS,
                        DU_NM,DU_NS,DU_ZE,
                        DU_NS,DU_ZE,DU_PS,
                        DU_ZE,DU_PS,DU_PM,
                        DU_PS,DU_PM,DU_PB};
    
    int du_fuzzy = 0;
    
    SpeedError = UALTHM_SetSpeed - UENC_GetSpeed();
    ChangeSpeedError = SpeedError - Pre_SpeedError;
    Pre_SpeedError = SpeedError;
    
    x1 = SpeedError * 100 / Fuzzy_Parameter.Ge;
    x2 = ChangeSpeedError * 100 / UALTHM_TIME_SAMPLING / Fuzzy_Parameter.Gde;
    
    if (x1>100)            x1 = 100;
    else if (x1<-100)      x1 = -100;
    
    if (x2>100)            x2 = 100;
    else if (x2<-100)      x2 = -100;
    //Old 
//    x1_NB = UFUZZY_Trap_mf(x1,-200,-100,-50,-20);
//    x1_NS = UFUZZY_Trap_mf(x1,-50,-20,-20,0);
//    x1_ZE = UFUZZY_Trap_mf(x1,-20,0,0,20);
//    x1_PS = UFUZZY_Trap_mf(x1,0,20,20,50);
//    x1_PB = UFUZZY_Trap_mf(x1,20,50,100,200);
//    
//    x2_NE = UFUZZY_Trap_mf(x2,-200,-100,-30,0);
//    x2_ZE = UFUZZY_Trap_mf(x2,-30,0,0,30);
//    x2_PO = UFUZZY_Trap_mf(x2,0,30,100,200);


    x1_NB = UFUZZY_Trap_mf(x1,-200,-100,-100,-40);
    x1_NS = UFUZZY_Trap_mf(x1,-100,-40,-40,0);
    x1_ZE = UFUZZY_Trap_mf(x1,-40,0,0,40);
    x1_PS = UFUZZY_Trap_mf(x1,0,40,40,100);
    x1_PB = UFUZZY_Trap_mf(x1,40,100,100,200);
    
    x2_NE = UFUZZY_Trap_mf(x2,-200,-100,-100,0);
    x2_ZE = UFUZZY_Trap_mf(x2,-100,0,0,100);
    x2_PO = UFUZZY_Trap_mf(x2,0,100,100,200);
    
    /*
    Percentage_DC.value = (float)x2;
    Des_Speed.value = (float)x2_NE;
    r_Speed.value = (float)(x2_ZE);
    Current.value = (float)(x2_PO);
    */
    
    fuzzy_value[0] = min(x1_NB,x2_NE);
    fuzzy_value[1] = min(x1_NB,x2_ZE);
    fuzzy_value[2] = min(x1_NB,x2_PO);
    fuzzy_value[3] = min(x1_NS,x2_NE);
    fuzzy_value[4] = min(x1_NS,x2_ZE);
    fuzzy_value[5] = min(x1_NS,x2_PO);
    fuzzy_value[6] = min(x1_ZE,x2_NE);
    fuzzy_value[7] = min(x1_ZE,x2_ZE);
    fuzzy_value[8] = min(x1_ZE,x2_PO);
    fuzzy_value[9] = min(x1_PS,x2_NE);
    fuzzy_value[10] = min(x1_PS,x2_ZE);
    fuzzy_value[11] = min(x1_PS,x2_PO);
    fuzzy_value[12] = min(x1_PB,x2_NE);
    fuzzy_value[13] = min(x1_PB,x2_ZE);
    fuzzy_value[14] = min(x1_PB,x2_PO);
    
    for (i=0;i<15;i++) du_fuzzy += du_value[i] * fuzzy_value[i];
    
    _DutyCycle += (int)( ( ( (long int)(du_fuzzy) ) * UALTHM_TIME_SAMPLING ) * Fuzzy_Parameter.Gdu ); 
		
		if(_DutyCycle <= 0)
		{
			_DutyCycle = 0;
			UPWM_StopPWM();
			UPWM_SetDutyCycle(_DutyCycle);
		}
		else
		{
			if(_DutyCycle < PWM_DC_MIN) _DutyCycle = PWM_DC_MIN;
			else if(_DutyCycle > PWM_DC_MAX) _DutyCycle = PWM_DC_MAX;
			
			UHALL_ReadPosition(_state_Motor);
			UPWM_SetDutyCycle(_DutyCycle);			
		}
}

void UALTHM_FUZZY_P(void)
{
	  int ChangePositionError;
    int i;
    int x1,x2;
    int x1_NB,x1_NS,x1_ZE,x1_PS,x1_PB;
    int x2_NE,x2_ZE,x2_PO;
    
    int fuzzy_value[15];
    int du_value[15] = {DU_NB,DU_NM,DU_NS,
                        DU_NM,DU_NS,DU_ZE,
                        DU_NS,DU_ZE,DU_PS,
                        DU_ZE,DU_PS,DU_PM,
                        DU_PS,DU_PM,DU_PB};
    
    int du_fuzzy = 0;
    
    SpeedError = UALTHM_SetSpeed - UENC_GetSpeed();
    ChangePositionError = PositionError - Pre_PositionError;
    Pre_PositionError = PositionError;
    
    x1 = PositionError * 100 / Fuzzy_Parameter.Ge;
    x2 = ChangePositionError * 100 / UALTHM_TIME_SAMPLING / Fuzzy_Parameter.Gde;
    
    if (x1>100)            x1 = 100;
    else if (x1<-100)      x1 = -100;
    
    if (x2>100)            x2 = 100;
    else if (x2<-100)      x2 = -100;
    //Old 
//    x1_NB = UFUZZY_Trap_mf(x1,-200,-100,-50,-20);
//    x1_NS = UFUZZY_Trap_mf(x1,-50,-20,-20,0);
//    x1_ZE = UFUZZY_Trap_mf(x1,-20,0,0,20);
//    x1_PS = UFUZZY_Trap_mf(x1,0,20,20,50);
//    x1_PB = UFUZZY_Trap_mf(x1,20,50,100,200);
//    
//    x2_NE = UFUZZY_Trap_mf(x2,-200,-100,-30,0);
//    x2_ZE = UFUZZY_Trap_mf(x2,-30,0,0,30);
//    x2_PO = UFUZZY_Trap_mf(x2,0,30,100,200);


    x1_NB = UFUZZY_Trap_mf(x1,-200,-100,-100,-40);
    x1_NS = UFUZZY_Trap_mf(x1,-100,-40,-40,0);
    x1_ZE = UFUZZY_Trap_mf(x1,-40,0,0,40);
    x1_PS = UFUZZY_Trap_mf(x1,0,40,40,100);
    x1_PB = UFUZZY_Trap_mf(x1,40,100,100,200);
    
    x2_NE = UFUZZY_Trap_mf(x2,-200,-100,-100,0);
    x2_ZE = UFUZZY_Trap_mf(x2,-100,0,0,100);
    x2_PO = UFUZZY_Trap_mf(x2,0,100,100,200);
    
    /*
    Percentage_DC.value = (float)x2;
    Des_Speed.value = (float)x2_NE;
    r_Speed.value = (float)(x2_ZE);
    Current.value = (float)(x2_PO);
    */
    
    fuzzy_value[0] = min(x1_NB,x2_NE);
    fuzzy_value[1] = min(x1_NB,x2_ZE);
    fuzzy_value[2] = min(x1_NB,x2_PO);
    fuzzy_value[3] = min(x1_NS,x2_NE);
    fuzzy_value[4] = min(x1_NS,x2_ZE);
    fuzzy_value[5] = min(x1_NS,x2_PO);
    fuzzy_value[6] = min(x1_ZE,x2_NE);
    fuzzy_value[7] = min(x1_ZE,x2_ZE);
    fuzzy_value[8] = min(x1_ZE,x2_PO);
    fuzzy_value[9] = min(x1_PS,x2_NE);
    fuzzy_value[10] = min(x1_PS,x2_ZE);
    fuzzy_value[11] = min(x1_PS,x2_PO);
    fuzzy_value[12] = min(x1_PB,x2_NE);
    fuzzy_value[13] = min(x1_PB,x2_ZE);
    fuzzy_value[14] = min(x1_PB,x2_PO);
    
    for (i=0;i<15;i++) du_fuzzy += du_value[i] * fuzzy_value[i];
    
    _DutyCycle += (int)( ( ( (long int)(du_fuzzy) ) * UALTHM_TIME_SAMPLING ) * Fuzzy_Parameter.Gdu ); 
		
//		if(_DutyCycle <= 0)
//		{
//			_DutyCycle = 0;
//			UPWM_StopPWM();
//			UPWM_SetDutyCycle(_DutyCycle);
//		}
//		else
//		{
//			if(_DutyCycle < PWM_DC_MIN) _DutyCycle = PWM_DC_MIN;
//			else if(_DutyCycle > PWM_DC_MAX) _DutyCycle = PWM_DC_MAX;
//			
//			UHALL_ReadPosition(_state_Motor);
//			UPWM_SetDutyCycle(_DutyCycle);			
//		}
		if (_DutyCycle>PWM_DC_MIN){
			UCAN_SetFlag(Flag_Motor_Run_CW);
			UHALL_ReadPosition(Motor_CW);
			UPWM_SetDutyCycle(_DutyCycle);
		}
		else if (_DutyCycle<-PWM_DC_MIN){
			UCAN_SetFlag(Flag_Motor_Run_CCW);
			UHALL_ReadPosition(Motor_CCW);
			UPWM_SetDutyCycle(-_DutyCycle);
		}
		else {
			_DutyCycle = 0;
			UPWM_SetDutyCycle(0);
			UPWM_StopPWM();
		}
}

int UFUZZY_Trap_mf(int x,int a,int b,int c,int d)
{
	int value;
  if (x<a) value = 0;
  else if (x<b) value = 100*(x-a)/(b-a);
  else if (x<c) value = 100;
  else if (x<d) value = 100*(d-x)/(d-c);
  else value = 0;
  return value;
}

void UALTHM_Controller(bool _state_Motor)
{
	if(UCAN_GetFlag(Flag_Open_Loop) == 0)
	{
		if(UCAN_GetFlag(Flag_PID_Control))
		{
			if (UCAN_GetFlag(Flag_Motor_Position)){
				UALTHM_PID_v3();
//				ULED_Toggle(ORANGE);
			}
			else
				UALTHM_PID_v2(_state_Motor);
		}
		else if(UCAN_GetFlag(Flag_Fuzzy_Control))
		{
			if (!UCAN_GetFlag(Flag_Motor_Position))
				UALTHM_FUZZY(_state_Motor);
			else {
				UALTHM_FUZZY_P();
			}
		}
		if((UPWM_GetRefDutyCycle() < 5.0))
		{
			UPWM_StopPWM();
			UPWM_SetDutyCycle(0);
		}
	}
}

uint32_t UALTHM_SetPointFilter(uint32_t setPoint){
	//0.2
//		LowPassFilter_Parameter.a = 0.04877f;
//		LowPassFilter_Parameter.b = 0.9512f;
	//0.1
	LowPassFilter_Parameter.a = 0.09516f;
	LowPassFilter_Parameter.b = 0.9048f;
	
//	LowPassFilter_Parameter.a = 0.01242f;
//	LowPassFilter_Parameter.b = 0.9876f;
	 
	_uk = setPoint;
	_yk = (float)LowPassFilter_Parameter.b*_yk1 + (float)LowPassFilter_Parameter.a*_uk1;
	_uk1 = _uk;
	_yk1 = _yk;
	if (setPoint - _yk < 100) _yk = setPoint;
	return _yk;
}

/******************************************************************************/
/* Peripherals Interrupt Handlers --------------------------------------------*/
/******************************************************************************/
