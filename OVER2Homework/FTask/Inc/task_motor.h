#ifndef __TASKMOTOR_
#define __TASKMOTOR_

#include "can.h"
#include "uart.h"
#include "sysconfig.h"
//#include "dianji_driver.h"
/*
typedef enum 
{
    Switch_Left,
    Switch_Right,
		swcounter
}rc_switch;
*/

extern QueueHandle_t Queue_CANSend;
//  Motor_Run(void);
void Task_MotorControl(void *parameters);
void Motor_Control(void);
void Motor_Protect(void);
void Motor_Stop(void);
switch_state Get_Switch_Value(rc_switch sw);
#endif  

