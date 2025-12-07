#ifndef __SYSCONFIG__H_
#define __SYSCONFIG__H_

/**Include Header Files**/
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "main.h"
#include "stm32h7xx_hal.h"
#include "fdcan.h"
#include "gpio.h"
#include "usart.h"
//#include "tim.h"
//#include "i2c.h"
//#include "adc.h"

#include "cmsis_os.h"
#include "task.h"


/********** Private  Macro *********/
#define Toggle_LED_Green()  HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define TurnOn_LED_Green()  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_RESET)
#define TurnOff_LED_Green()  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port,LED_GREEN_Pin,GPIO_PIN_SET)

#define Toggle_LED_Red()    HAL_GPIO_TogglePin(LED_RED_GPIO_Port,LED_RED_Pin)
#define TurnOn_LED_Red()  HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_RESET)
#define TurnOff_LED_Red()  HAL_GPIO_WritePin(LED_RED_GPIO_Port,LED_RED_Pin,GPIO_PIN_SET)

#define Power_GPIO_Port GPIOA
#define Power_Pin GPIO_PIN_1//暂且这么写，之后必须删掉
/********** Mode Select **********/
#define DEBUG_MODE
//#define RELEASE_MODE

#ifdef DEBUG_MODE
#define MODE_SELECTED
#endif

#ifdef RELEASE_MODE
#define MODE_SELECTED
#endif

#ifndef MODE_SELECTED
#error Please select your current mode(Debug or Release)
#endif

/************* Uart buffer ***************/
//串口缓存区
#define MEMORY0 0
#define MEMORY1 1
#define MEMORYRESET 2

/********** Math limit **********/
#define LIMIT(data,min,max) (data = data > max ? max : (data < min ? min : data))
#define PI 3.14159265f

#define CANSEND_1 1
#define CANSEND_2 2
#define CANSEND_3 3


//CAN 1
#define DJI_down_id			3
#define LK_mid_id				2

// CAN 2
#define DJI_up_id      	1

typedef struct
{
    uint8_t            CANx;               
    uint32_t           stdid;              
		uint8_t            Data[8];
}CanSend_Type;

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

#endif
