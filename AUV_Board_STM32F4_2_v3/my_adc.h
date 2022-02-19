#ifndef _MY_ADC_H_
#define _MY_ADC_H_

#include "misc.h"
#include "stm32f4xx_can.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_tim.h"
#include "stm32f4xx_adc.h"
#include <stdbool.h>
#include "my_mx28.h"
#include "my_bms24v40ah.h"
#include "my_keller_pa3.h"
#include "my_io.h"
#include "my_can.h"
#include "my_delay.h"

extern volatile uint32_t xsen_angle;

void ADC_Config(void);

#endif
