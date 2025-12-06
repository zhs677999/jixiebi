#ifndef __TASK_INIT_H_
#define __TASK_INIT_H_

#include "sysconfig.h"
#include "TaskCan.h"
//#include "TaskUART.h"
#include "TaskMOTOR.h"
#include "TaskState.h"

extern QueueHandle_t Queue_CANSend;
//ÐÂ¼Ó
extern RCDecoding_Type Remote_controler;
extern uint8_t RCBuffer[2][RC_FRAME_LEN + RC_FRAME_LEN_BACK];
extern uint8_t UartReceive_Data[UartCommLen];


#endif
