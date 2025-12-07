#include "TaskMOTOR.h"


#define TASK_MOTORCONTROL_INTERVAL (1)

int ctrlcount = 0;
uint8_t MotorCount = 0;
uint8_t DM_TurnCount = 0;
uint8_t CAN1_TurnCount = 0;
uint8_t CAN2_TurnCount = 0;
uint8_t Protect_TurnCount = 0;
uint8_t ZeroFlag = 0;
uint8_t was_running = 0;

extern motor_t YawB,PitchMid;
extern motor_t DJIup;

#define CANSEND_1 1
#define CANSEND_2 2
#define CANSEND_3 3


void Task_MotorControl(void *parameters)
{
    TickType_t xLastWakeUpTime = xTaskGetTickCount();

    MotionMotor_Init();   // PID、电机参数初始化，自己写的，定义在dianjiDriver

	xLastWakeUpTime = xTaskGetTickCount();
	// HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); 大作业用不到
	while (1)
	{

		ctrlcount++;/*
		if (ZeroFlag != 0)
			Motor_SetZero(&ZeroFlag);

		start_time2 = xTaskGetTickCount();

		if (MotorCount % 10 == 0)
		{
			MCU_Control_CMD(); // 双板通信发送数据更新
			if (uart_first_rx_flag)
			{
				Uart_Send(MotorCount); // 双板通信硬件发送
			}
			Uart_Comm_Receive(); // 双板通信掉线检测
		}
*/
		if (Motor_Define == 1&& Get_Switch_Value(Switch_Right) ==sw_mid ) // 记得写！
		//if (Motor_Define == 1)
		{
			MotorCount++;
			CAN1_TurnCount++;
			CAN2_TurnCount++;
			// CAN3_TurnCount++;

			//if (GetSubState() != SubState_Protect)
			was_running = 1;
			Motor_Control();
			//Servo_anglecontrol();
				 
			}
		else
			{
				 if (was_running)
    {
        Motor_Stop();
        Motor_Protect();
        was_running = 0;
    }
			}
		 
		vTaskDelayUntil(&xLastWakeUpTime, TASK_MOTORCONTROL_INTERVAL);
	}
}


    
int LK_read=0;
    

void Motor_Control(void)
{
		YawB_AngleSpeedCurrent_Loop(&YawB);
		PitchMid_AngleSpeedCurrent_Loop(&PitchMid);
	{
	case 3:
	{
		DJIdown_AngleSpeedCurrent_Loop(&DJIdown);
		//DM_speedpositionControl(RollB_Motor_id, CANSEND_1, RollB_Motor.TargetAngle + 360 * RollB_Motor.Targetcirnum, 50);
		break;
	}
	case 1:
	{
		LKmid_AngleSpeedCurrent_Loop(&LKmid);
		break;
	}
	/*case 0:
	{
		DM_speedpositionControl(PitchF_Motor_id, CANSEND_1, PitchF_Motor.TargetAngle + 360 * PitchF_Motor.Targetcirnum, 20);
		break;
	}*/
	default:
		LK_MultiLoop_angleRead(LK_mid_id, CANSEND_1);
		LK_read++;
//		Update_motor();
		
		break;
	}
	/*——————————CAN2———————————*/
	switch (CAN2_TurnCount % 3)
	{
	case 1:
	{
		DJIup_AngleSpeedCurrent_Loop(&DJIup);
		break;
	}
	case 0:
	{
		/*RSrun_count[1]++;
		RS_indexWrite(PitchB_Motor_id, 0x7016, (PitchB_Motor.TargetAngle + 360 * PitchB_Motor.Targetcirnum) / 180 * 3.1415926f, CANSEND_2);
		break;
		*/
	}
	default:
	//	Update_motor();
		break;
	}
}

/*
switch (CAN3_TurnCount % 5)
	{
	case 0:
	{
		if(hPCState.PC_MainState==Sell_Mode &&hPCState.sell_slavestate==Sell_Get_State &&hPCState.ministate==State_0)
		{
			yaw_max_speed=200;
		}
		else
		{
			yaw_max_speed=100;
		}
		LK_MultiLoop_angleControl_limited(yaw_max_speed, Yaw_Motor.Targetcirnum * 360.0f + Yaw_Motor.TargetAngle, Yaw_Motor_id, CANSEND_3, 6);
		break;
	}
	default:
		LK_MultiLoop_angleRead(Yaw_Motor_id, CANSEND_3);
		break;
	}
*/


/**
    YawB.TargetAngle  = YawB.RealAngle;
    YawB.Targetcirnum = YawB.Realcirnum;
    PitchMid.TargetAngle    = PitchMid.RealAngle;
    PitchMid.Targetcirnum   = PitchMid.Realcirnum;
 */

void Motor_Protect(void)
{
    Motor_Define = 0;   

    DJIdown.TargetAngle  = DJIdown.RealAngle;
    DJIdown.Targetcirnum = DJIdown.Realcirnum;

    LKmid.TargetAngle    = LKmid.RealAngle;
    LKmid.Targetcirnum   = LKmid.Realcirnum;

    DJIup.TargetAngle    = DJIup.RealAngle;
    DJIup.Targetcirnum   = DJIup.Realcirnum;

}

void Motor_Stop(void)
{
    static CanSend_Type CANSend;

    // 选择哪路 CAN，这里先以 CAN1 为例
    CANSend.CANx  = CANSEND_1;
    // 和 DJI_GM6020_CurrentControl 一样，用电流控制帧 ID
    CANSend.stdid = GM6020_CURRENT_CTRL;   // 一般是 0x1FE

    // 8 字节全 0，对应 4 个电机电流全 0
    CANSend.Data[0] = 0;
    CANSend.Data[1] = 0;
    CANSend.Data[2] = 0;
    CANSend.Data[3] = 0;
    CANSend.Data[4] = 0;
    CANSend.Data[5] = 0;
    CANSend.Data[6] = 0;
    CANSend.Data[7] = 0;

    // 入队，由 Task_CAN 统一发送
    xQueueSend(Queue_CANSend, &CANSend, 3);
	    

    // 选择哪路 CAN，这里先以 CAN1 为例
    CANSend.CANx  = CANSEND_2;
    // 和 DJI_GM6020_CurrentControl 一样，用电流控制帧 ID
    CANSend.stdid = GM6020_CURRENT_CTRL;   // 一般是 0x1FE

    // 8 字节全 0，对应 4 个电机电流全 0
    CANSend.Data[0] = 0;
    CANSend.Data[1] = 0;
    CANSend.Data[2] = 0;
    CANSend.Data[3] = 0;
    CANSend.Data[4] = 0;
    CANSend.Data[5] = 0;
    CANSend.Data[6] = 0;
    CANSend.Data[7] = 0;

    // 入队，由 Task_CAN 统一发送
    xQueueSend(Queue_CANSend, &CANSend, 3);
		LK_iqControl(0, LK_mid_id, CANSEND_1);}
/*

效果：打开电机


void Motor_Run(void)
{
	run_count++;
	——————————CAN1———————————

	if (RollB_Run == 0)
	{
		DM_motorOpen(RollB_Motor_id, CANSEND_1);
		RollB_Run++;
	}
	else if (RollF_Run == 0)
	{
		DM_motorOpen(RollF_Motor_id, CANSEND_1);
		RollF_Run++;
	}
	else if (PitchF_Run == 0)
	{
		DM_motorOpen(PitchF_Motor_id, CANSEND_1);
		PitchF_Run++;
	}

	——————————CAN2———————————
	if (RSB_SetMode == 1)
	{
		RS_indexRead(PitchB_Motor_id, RS_MOTOR_MODE, CANSEND_2);
		RSB_SetMode = 0;
	}
	else if (RSM_SetMode == 1)
	{
		RS_indexRead(PitchM_Motor_id, RS_MOTOR_MODE, CANSEND_2);
		RSM_SetMode = 0;
	}
	else
	{
		if (PitchB_Motor.Mode != 1)
		{
			RS_indexWrite(PitchB_Motor_id, RS_MOTOR_MODE, 1, CANSEND_2);
			RSB_SetMode = 1;
		}
		else if (PitchM_Motor.Mode != 1)
		{
			RS_indexWrite(PitchM_Motor_id, RS_MOTOR_MODE, 1, CANSEND_2);
			RSM_SetMode = 1;
		}
		else
		{
			if (!PitchM_Run)
			{
				RS_Motor_run(PitchM_Motor_id, CANSEND_2);
				PitchM_Run = 1;
				PitchM_Run_Tick = xTaskGetTickCount();
			}
			else if (!PitchB_Run)
			{
				RS_Motor_run(PitchB_Motor_id, CANSEND_2);
				PitchB_Run = 1;
			}
		}
	}

	——————————CAN3———————————
	if (PitchB_Run && PitchM_Run && RollB_Run && PitchF_Run && RollF_Run)
	{
		Motor_Work = 1;
	}
}*/
