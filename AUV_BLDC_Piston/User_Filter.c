/* Includes ------------------------------------------------------------------*/
#include "User_Filter.h"
/* External functions define -------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/


/* Private const/macros ------------------------------------------------------*/

/**
 * @defgroup Timer define
 * @{
 */
			/** 
			* @brief   Timer delay define
			*/


/**
 * @}
 */
 

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/


/* Exported function body ----------------------------------------------------*/
float UFIL_1stOrderFilter_Output(FirstOrderParameter* myFilter, float _uk){
	static float _uk1 = 0, _yk1 = 0, _yk = 0;
	_yk = (float)myFilter->b*_yk1 + (float)myFilter->a*_uk1;
	_uk1 = _uk;
	_yk1 = _yk;
	return _yk;
}
float UFIL_2ndOrderFilter_Output(SecondOrderParameter* myFilter, float _uk){
	static float _uk_1 = 0,_uk_2 = 0, _yk_1 = 0, _yk_2= 0, _yk=0;
    _yk = myFilter->a*_uk_1 + myFilter->b*_uk_2 - myFilter->c*_yk_1 - myFilter->d*_yk_2;
    _yk_1 = _yk;
    _yk_2 = _yk_1;
    _uk_1 = _uk;
    _uk_2 = _uk_1;
	return _yk;
}