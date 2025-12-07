#ifndef TASK_RUNTIME_H
#define TASK_RUNTIME_H

#include "control_system.h"

#define TASK_STATEMACHINE_INTERVAL (15)
#define TASK_MOTORCONTROL_INTERVAL pdMS_TO_TICKS(10)

void Task_CAN(void *parameters);
void CAN_Init(FDCAN_HandleTypeDef *hfdcan);
void CAN_Fliter_Init(FDCAN_HandleTypeDef *hfdcan);
void CANTransmit(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, uint8_t aData[]);

void Task_MotorControl(void *parameters);
void Motor_Control(void);
void Motor_Protect(void);
void Motor_Stop(void);

void Task_StateMachine(void *parameters);
void Switch_Update(void);

void TaskInit(void const *parameters);

extern QueueHandle_t Queue_CANSend;
extern TaskHandle_t CAN_TASK_Handle;
extern TaskHandle_t MOTORCONTROL_TASK_Handle;
extern TaskHandle_t STATEMACHINE_TASK_Handle;

#endif
