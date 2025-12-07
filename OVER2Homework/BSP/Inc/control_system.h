#ifndef CONTROL_SYSTEM_H
#define CONTROL_SYSTEM_H

#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "main.h"
#include "fdcan.h"
#include "gpio.h"
#include "usart.h"
#include "cmsis_os.h"
#include "task.h"

#define Toggle_LED_Green() HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin)
#define TurnOn_LED_Green() HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_RESET)
#define TurnOff_LED_Green() HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, GPIO_PIN_SET)
#define Toggle_LED_Red() HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin)
#define TurnOn_LED_Red() HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_RESET)
#define TurnOff_LED_Red() HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, GPIO_PIN_SET)

#define MEMORY0 0
#define MEMORY1 1
#define MEMORYRESET 2
#define LIMIT(data, min, max) (data = data > max ? max : (data < min ? min : data))
#define PI_VALUE 3.14159265f

#define CANSEND_1 1
#define CANSEND_2 2
#define CANSEND_3 3

#define GM6020_FEEDBACK_ID 0x204U
#define GM6020_CURRENT_CTRL 0x1FEU

#define RC_HUART huart5
#define RC_CH_MAX_RELATIVE 660.0f
#define RC_FRAME_LEN 18U
#define RC_FRAME_LEN_BACK 2U
#define RC_CH_VALUE_MIN ((uint16_t)364)
#define RC_CH_VALUE_OFFSET ((uint16_t)1024)
#define RC_CH_VALUE_MAX ((uint16_t)1684)
#define RC_CH0 ((uint8_t)0)
#define RC_CH1 ((uint8_t)1)
#define RC_CH2 ((uint8_t)2)
#define RC_CH3 ((uint8_t)3)
#define RC_CH4 ((uint8_t)4)
#define RC_SW_UP ((uint16_t)1)
#define RC_SW_MID ((uint16_t)3)
#define RC_SW_DOWN ((uint16_t)2)
#define RC_SW_Right ((uint8_t)0)
#define RC_SW_Left ((uint8_t)1)

#define UARTCOMM_HUART huart5
#define Frame_Head1 0x11
#define Frame_Head2 0x12
#define Frame_Tail 0x14
#define UartCommLen 18

typedef enum {
    MOTOR_NONE,
    MOTOR_2006,
    MOTOR_3508,
    MOTOR_6020,
    MOTOR_6623,
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
} motor_type;

typedef enum {
    PID_POSITION = 0,
    PID_DELTA
} PID_MODE;

typedef struct {
    PID_MODE pidmode;
    float error[3];
    float sum_error;
    float Dbuf[3];
    float kp;
    float ki;
    float kd;
    float Pout;
    float Iout;
    float Dout;
    float Out;
    float dst;
    float real;
    float MaxOut;
    float MaxIout;
    float MaxBlockIount;
    uint16_t DeadZone;
} pid_type;

typedef struct {
    uint32_t FrameCounter;
    int16_t RealSpeed;
    int16_t Current;
    float test;
    int16_t Voltage;
    uint16_t Mechanical_Angle[2];
    int16_t Temperature;
} Motor_Feedback_Data;

typedef struct {
    motor_type type;
    uint32_t id;
    uint8_t Error_id;
    uint8_t Mode;
    pid_type *PositionPID;
    pid_type *SpeedPID;
    Motor_Feedback_Data FeedbackData;
    uint8_t Reductionratio;
    int16_t Oricirnum;
    int16_t Realcirnum;
    int16_t Targetcirnum;
    int16_t Realrotationrate;
    float Targetrotationrate;
    float InitAngle;
    float RealAngle;
    float TargetAngle;
    int16_t TargetMechAngle;
    int16_t Out;
    uint8_t First_Frame;
} motor_t;

typedef struct {
    uint8_t CANx;
    uint32_t stdid;
    uint8_t Data[8];
} CanSend_Type;

typedef enum {
    CH_RightHori,
    CH_RightVert,
    CH_LeftHori,
    CH_LeftVert,
    CH_Roll
} channel_num;

typedef enum {
    Switch_Left,
    Switch_Right,
    swcounter
} rc_switch;

typedef enum {
    sw_offset,
    sw_up = 1,
    sw_down,
    sw_mid
} switch_state;

typedef enum {
    Roller_Mid = 0,
    Roller_Small_Front,
    Roller_Big_Front,
    Roller_Small_Back,
    Roller_Big_Back
} Roller_State_t;

typedef enum {
    Stick_Mid_V = 0,
    Stick_Small_Front,
    Stick_Big_Front,
    Stick_Small_Back,
    Stick_Big_Back
} Stick_Vert_t;

typedef enum {
    Stick_Mid_H = 0,
    Stick_Small_Right,
    Stick_Big_Right,
    Stick_Small_Left,
    Stick_Big_Left
} Stick_Hori_t;

typedef enum {
    UnPressed = 0,
    Pressed,
} Key_Value_t;

typedef struct {
    uint8_t SoH;
    uint32_t FrameCounter;
    uint8_t GimbalMotor_BlockFinish;
    uint8_t BranchMotor_BlockFinish;
    int32_t GimbalL_RealAngle;
    uint8_t EoT;
} UartReceive_Data_t1;

typedef struct {
    uint8_t SoH;
    int32_t BranchL_RealAngle;
    int32_t BranchF_RealAngle;
    int32_t BranchP_RealAngle;
    uint8_t EoT;
} UartReceive_Data_t2;

typedef __packed struct {
    __packed struct {
        __packed struct {
            switch_state sw[2];
            uint16_t ch[5];
        } rc;
        __packed struct {
            int16_t x;
            int16_t y;
            int16_t z;
            uint8_t press_l;
            uint8_t press_r;
        } mouse;
        __packed struct {
            uint16_t key_bit;
        } keyboard;
    } buffer[2];
    uint8_t buffer_index;
    uint8_t offline_check;
    int32_t framecounter;
    uint8_t swtrigger[2];
    uint8_t mousetrigger[2];
    uint16_t mousePresstime[2];
    uint16_t keyPresstime[16];
    uint16_t keytrigger;
} RCDecoding_Type;

typedef struct {
    uint32_t system_tick;
    uint32_t can_tx_count;
    uint32_t can_rx_count;
    uint32_t last_error;
    uint8_t test_phase;
    uint8_t any_motor_online;
    uint32_t phase_start_time;
} Debug_Info_t;

extern RCDecoding_Type Remote_controler;
extern uint8_t RCBuffer[2][RC_FRAME_LEN + RC_FRAME_LEN_BACK];
extern uint8_t UartReceive_Data[UartCommLen];
extern motor_t YawB;
extern motor_t PitchMid;
extern motor_t DJIdown;
extern motor_t LKmid;
extern int Motor_Define;
extern Debug_Info_t debug_info;

int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);

uint8_t PID_Parameter_Init(pid_type *PID, PID_MODE mode, float Pidpara[], uint32_t MaxOut, uint32_t MaxIout, uint16_t DeadZone);
uint8_t PID_Parameter_Update(pid_type *pid, float Pidpara[]);
uint8_t PID_clear(pid_type *pid);
float Pid_cal(pid_type *PID, float dst, float real);

void MotortypeRegister(motor_t *mot, motor_type type, uint8_t Reductionratio, uint32_t Can_id);
void MotortypeRegisterpid(motor_t *mot, motor_type type, pid_type *pospid, pid_type *speedpid, uint8_t Reductionratio, uint32_t Can_id);
void MotionMotor_Init(void);

void DJIMotor_ParaHandle(motor_t *mot, uint8_t adata[]);
int8_t DJIMotor_AngleHandle(motor_t *mot);
void LK_Motor_ParaHandle(motor_t *motor, uint8_t aData[]);
void Can1Received_infoHandle(uint32_t stdid, uint8_t adata[]);
void Can2Received_infoHandle(uint32_t stdid, uint8_t adata[]);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);

void DJI_GM6020_CurrentControl(int16_t cmd, uint8_t motor_id, uint8_t canx);
void DJI_AngleSpeedCurrent_Loop(motor_t *mot, uint8_t motor_id, uint8_t canx);
void YawB_AngleSpeedCurrent_Loop(motor_t *mot);
void PitchMid_AngleSpeedCurrent_Loop(motor_t *mot);
void LK_MultiLoop_angleRead(uint8_t Motor_ID, uint8_t CAN_ID);
void LK_MultiLoop_angleControl_limited(uint16_t max_speed, float angle_control_float, uint8_t Motor_ID, uint8_t CAN_ID, uint16_t reductionratio);
void LK_iqControl(int32_t iq_control, uint8_t Motor_ID, uint8_t CAN_ID);
void LKmid_AngleSpeedCurrent_Loop(motor_t *mot);
void DJIdown_AngleSpeedCurrent_Loop(motor_t *mot);

void Updata_Trigger(void);
uint8_t RcDataUpdate(void);
void BSP_Init_RemoteControl(void);
void RC_IRQHandler(UART_HandleTypeDef *huart);
void Update_motor(void);
void Angle_Limit(motor_t *Motor);
void Motor_Set_TargetAngle(motor_t *m, float relTargetAngle);
void Motor_Add_TargetAngle(motor_t *m, float AddAngle);
float Get_Channel_Value(channel_num channel);
switch_state Get_Switch_Value(rc_switch sw);

float MechanicalAngle2RealAngle(uint16_t Mechanical_Angle);

#endif
