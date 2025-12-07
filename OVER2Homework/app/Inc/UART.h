#ifndef _UART__H__
#define _UART__H__

#include "motor_driver.h"
// #include "sysconfig.h"  电机.h中有

//下面是RC相关定义

#define RC_HUART huart5

#define RC_CH_MAX_RELATIVE 660.0f				//遥控器通道相对最大值

#define RC_FRAME_LEN        18U         //遥控器数据帧长
#define RC_FRAME_LEN_BACK   2U          //增加两个字节保持稳定

/* ----------------------- RC Channel Definition---------------------------- */
#define RC_CH_VALUE_MIN ((uint16_t)364 )
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH0    ((uint8_t)0)
#define RC_CH1    ((uint8_t)1)
#define RC_CH2    ((uint8_t)2)
#define RC_CH3    ((uint8_t)3)
#define RC_CH4    ((uint8_t)4)
/* ----------------------- RC Switch Definition----------------------------- */
#define RC_SW_UP    ((uint16_t)1)
#define RC_SW_MID   ((uint16_t)3)
#define RC_SW_DOWN  ((uint16_t)2)
#define RC_SW_Right ((uint8_t)0)
#define RC_SW_Left  ((uint8_t)1)

/* ------------------------------- Enum types ------------------------------- */

/*  在motor_driver.h定义
typedef enum 
{
    CH_RightHori,   //right horizon;	channel0
    CH_RightVert,   //right vertical;	channel1
    CH_LeftHori,   	//left  horizon;	channel2
    CH_LeftVert,   	//left  vertical;	channel3
		CH_Roll					//left wheel;			channel4
}channel_num;
*/
typedef enum 
{
    Switch_Left,
    Switch_Right,
		swcounter
}rc_switch;

typedef enum
{
    sw_offset, //未赋值
    sw_up = 1,
    sw_down,
    sw_mid
}switch_state;



typedef struct
{
	uint8_t SoH;
	uint32_t FrameCounter;

	uint8_t GimbalMotor_BlockFinish;
	uint8_t BranchMotor_BlockFinish;

	int32_t GimbalL_RealAngle;

	uint8_t EoT;
}UartReceive_Data_t1;

typedef struct
{
	uint8_t SoH;

	int32_t BranchL_RealAngle;
	int32_t BranchF_RealAngle;
	int32_t BranchP_RealAngle;

	uint8_t EoT;
}UartReceive_Data_t2;

typedef __packed struct
{
    uint8_t buffer_index;  // 当前使用的缓存区
    uint8_t offline_check; // 离线计数，用于判断遥控是否离线

    int32_t framecounter; // 用于计数包数

    uint8_t swtrigger[2];       // 拨码开关切换状态
    uint8_t mousetrigger[2];    // 鼠标切换状态
    uint16_t mousePresstime[2]; // 鼠标按下时间

    uint16_t keyPresstime[16]; // 键盘按键按下时间
    uint16_t keytrigger;       // 键盘按键切换状态
    __packed struct
    {
        __packed struct
        {
            /* data */
            switch_state sw[2];
            uint16_t ch[5];
        } rc;

        __packed struct
        {
            /* data */
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t press_l;
            uint8_t press_r;
        } mouse;

        __packed struct
        {
            /* data */
            uint16_t key_bit;
        } keyboard;
    } buffer[2];

} RCDecoding_Type;




#define UARTCOMM_HUART 		huart5
#define Frame_Head1   			0x11
#define Frame_Head2   			0x12
#define Frame_Tail   			0x14
#define UartCommLen       	18


void Updata_Trigger(void);
uint8_t RcDataUpdate(void);
void BSP_Init_RemoteControl(void);
void RC_IRQHandler(UART_HandleTypeDef *huart);
void Update_motor(void);
void Angle_Limit(motor_t *Motor);
void Motor_Set_TargetAngle(motor_t* m, float relTargetAngle);
void Motor_Add_TargetAngle(motor_t* m, float AddAngle);
float Get_Channel_Value(channel_num channel);
switch_state Get_Switch_Value(rc_switch sw);







#endif
