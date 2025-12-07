#ifndef __DRIVER_
#define __DRIVER_

// #include "sysconfig.h"
#include "task_pid.h"

//下面是整体电机（对应Motor.h）

#define MechanicalAngle2RealAngle(Mechanical_Angle) (Mechanical_Angle / 8191.0f * 360.0f) //机械角（0~8191）->（0~360°）

typedef enum
{
    MOTOR_NONE,
	
		MOTOR_2006,
		MOTOR_3508,
		MOTOR_6020,
		MOTOR_6623,
	
		//LK电机
		MOTOR_4010,
	
	
		MOTOR_DM4310,
		MOTOR_DM4340,
		MOTOR_DM6220,
		
		MOTOR_MF4005,
		MOTOR_MG4005,
		MOTOR_MS5005,
    MOTOR_MG5005,
		MOTOR_MG8016,
	
		MOTOR_RS00,
		MOTOR_RS03,
		MOTOR_RS04,
	
		MOTOR_HT4315,
		MOTOR_HT9015,
    MOTOR_TYPE_COUNTER
}motor_type;

typedef struct
{
	uint32_t    FrameCounter;           //帧率计数
	int16_t    	RealSpeed;       			  //rpm
	int16_t    	Current;
	float test;
	int16_t			Voltage;
	uint16_t    Mechanical_Angle[2];    //电机返回机械角 [0]:当前 [1]:上一次
	
	int16_t Temperature;

}Motor_Feedback_Data;


typedef struct
{
	/* data */
	motor_type  type; 							//Motor_type
	uint32_t		id;									//Motor_ID
	uint8_t			Error_id;
	uint8_t			Mode;
	
	pid_type*   PositionPID;
	pid_type*   SpeedPID;
	Motor_Feedback_Data FeedbackData;
	
	uint8_t     Reductionratio; 		//减速比
	int16_t     Oricirnum;          //减速前转子转过圈数
	int16_t     Realcirnum;         //实际转过圈数
	int16_t     Targetcirnum;       //目标圈数

	int16_t     Realrotationrate;   //实际转速  rpm
	float       Targetrotationrate; //目标转速  rpm

	float   InitAngle;
	float   RealAngle;          //实际角度  单位°
	float   TargetAngle;        //目标角度  单位°
	int16_t TargetMechAngle;		//目标机械角度

	int16_t     	Out;                //PID计算后输出值
	
	//BlockStruct_t Blockstate;       //电机堵转状态
	
	uint8_t First_Frame;
		
}motor_t;

typedef enum 
{
    CH_RightHori,   //right horizon;	channel0
    CH_RightVert,   //right vertical;	channel1
    CH_LeftHori,   	//left  horizon;	channel2
    CH_LeftVert,   	//left  vertical;	channel3
		CH_Roll					//left wheel;			channel4
}channel_num;

void MotionMotor_Init(void);

extern int Motor_Define;
void PitchMid_AngleSpeedCurrent_Loop(motor_t *mot);
void YawB_AngleSpeedCurrent_Loop(motor_t *mot);
#define GM6020_FEEDBACK_ID   0x204U     // 反馈 ID = 0x204 + 电机ID




// 下面是LK电机
#define LK_STDID 0x140
#define LK_ENCODER_RESOLUTION  16384.0f  
void LK_MultiLoop_angleRead(uint8_t Motor_ID, uint8_t CAN_ID);
void LK_MultiLoop_angleControl_limited(uint16_t max_speed, float angle_control_float, uint8_t Motor_ID, uint8_t CAN_ID, uint16_t reductionratio);
void LK_iqControl(int32_t iq_control, uint8_t Motor_ID, uint8_t CAN_ID);
void DJI_AngleSpeedCurrent_Loop(motor_t *mot,uint8_t motor_id,uint8_t canx);
void DJI_GM6020_CurrentControl(int16_t cmd, uint8_t motor_id, uint8_t canx);
void LKmid_AngleSpeedCurrent_Loop(motor_t *mot);
void DJIdown_AngleSpeedCurrent_Loop(motor_t *mot);
void DJIup_AngleSpeedCurrent_Loop(motor_t *mot);
void MotortypeRegisterpid(motor_t *mot,
                       motor_type type,
                       pid_type *pospid,
                       pid_type *speedpid,
                       uint8_t Reductionratio,
                       uint32_t Can_id);

// void LK_Motor_ParaHandle(motor_t *mot, uint32_t stdid, uint8_t adata[]); can.c中定义了

#endif
