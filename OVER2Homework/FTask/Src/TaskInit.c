#include "TaskInit.h"


/*下面两个已经在TaskInit.h里引用的
#include "TaskCAN.h"
#include "TaskUART.h"

*/

#define CAN_PRIORITY 14
#define CAN_STACK_SIZE 512
TaskHandle_t CAN_TASK_Handle;

#define UART_PRIORITY 15
#define UART_STACK_SIZE 256
TaskHandle_t UART_TASK_Handle;

#define MOTORCONTROL_PRIORITY 12
#define MOTORCONTROL_STACK_SIZE 512
TaskHandle_t MOTORCONTROL_TASK_Handle;

#define STATEMACHINE_PRIORITY 13
#define STATEMACHINE_STACK_SIZE 256
TaskHandle_t STATEMACHINE_TASK_Handle;

QueueHandle_t Queue_CANSend;
int initcount = 0;

int bsp_start=0;
int taskcreated_all=0;

void TaskInit(void *parameters)
{
    initcount++;

    taskENTER_CRITICAL();
	
	// ai 认为加，实际上是拆了原来的BSP_Init
	Queue_CANSend = xQueueCreate(30, sizeof(CanSend_Type)); // 创建发送队列
	 BSP_Init_RemoteControl();
		CAN_Init(&hfdcan1);
		bsp_start++;
    CAN_Init(&hfdcan2);
	
		xTaskCreate((TaskFunction_t)Task_CAN,
                (const char *)"Task_CAN",
                (uint16_t)CAN_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)CAN_PRIORITY,
                (TaskHandle_t *)&CAN_TASK_Handle);

   /* xTaskCreate((TaskFunction_t)Task_UART,
                (const char *)"Task_UART",
                (uint16_t)UART_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)UART_PRIORITY,
                (TaskHandle_t *)&UART_TASK_Handle);  */
								
		xTaskCreate((TaskFunction_t)Task_MotorControl,
                (const char *)"Task_MotorControl",
                (uint16_t)MOTORCONTROL_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)MOTORCONTROL_PRIORITY,
                (TaskHandle_t *)&MOTORCONTROL_TASK_Handle);
								
		 xTaskCreate((TaskFunction_t)Task_StateMachine,
                (const char *)"Task_StateMachine",
                (uint16_t)STATEMACHINE_STACK_SIZE,
                (void *)NULL,
                (UBaseType_t)STATEMACHINE_PRIORITY,
                (TaskHandle_t *)&STATEMACHINE_TASK_Handle);
	
		//HAL_Delay(1000);
    vTaskDelete(NULL); // Delete Task_Init
		taskcreated_all++;
    taskEXIT_CRITICAL();


}
