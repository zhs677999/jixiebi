#ifndef __TASK_INIT_H_
#define __TASK_INIT_H_

#include "sysconfig.h"
#include "task_can.h"
//#include "Taskuart.h"
#include "task_motor.h"
#include "task_state.h"

extern QueueHandle_t Queue_CANSend;
//ÐÂ¼Ó
extern RCDecoding_Type Remote_controler;
extern uint8_t RCBuffer[2][RC_FRAME_LEN + RC_FRAME_LEN_BACK];
extern uint8_t UartReceive_Data[UartCommLen];


#endif
