#ifndef _TASK_STATEMACHINE_H__
#define _TASK_STATEMACHINE_H__


/**Include Header Files**/
//#include "sysconfig.h"
#include "Can.h"


/**Enum Definition**/

#define Big_Front    4
#define Big_Back     5
#define Small_Front  6
#define Small_Back   7

#define Big_Left    4
#define Big_Right     5
#define Small_Left  6
#define Small_Right   7


//State
typedef enum
{
    MainState_Debug = 0,				/*主状态——调试模式*/
    MainState_Vision,
    MainState_Extend,
		MainState_Counter
} MainState_t;

typedef enum
{
    SubState_Protect = 0,				/*从状态——保护模式*/
    SubState_Test,
    SubState_PC,

    SubState_Big_Joint,
    SubState_Small_Joint,
    SubState_XYZ,

    SubState_Gimbal,
    SubState_Branch,
    SubState_DIY,

    SubState_Counter						/*无意义，用来计数到底几个状态*/
} SubState_t;

typedef enum
{
    ControlMode_RC = 0,
    ControlMode_PC
} ControlMode_t;

typedef enum
{
    PCMode_Switch = 0,
    PCMode_Key
} PCMode_t;


//Key_Value
typedef enum
{
    Roller_Mid = 0,								/*拨杆居中*/
    Roller_Small_Front,
    Roller_Big_Front,							/*拨杆向前*/
    Roller_Small_Back,
    Roller_Big_Back								/*拨杆向后*/
} Roller_State_t;

typedef enum
{
    Stick_Mid_V = 0,					/*摇杆居中*/
    Stick_Small_Front,				/*摇杆小范围前摇*/
    Stick_Big_Front,					/*摇杆大范围前摇*/
    Stick_Small_Back,					/*摇杆小范围后摇*/
    Stick_Big_Back,						/*摇杆大范围后摇*/
} Stick_Vert_t;

typedef enum
{
    Stick_Mid_H = 0,					    /*摇杆居中*/
    Stick_Small_Right,           	/*摇杆小范围右摇*/
    Stick_Big_Right,							/*摇杆大范围右摇*/
    Stick_Small_Left,					    /*摇杆小范围左摇*/
    Stick_Big_Left,						    /*摇杆大范围左摇*/
} Stick_Hori_t;

typedef enum
{
    UnPressed = 0,
    Pressed,
} Key_Value_t;



/**Extern Declaration**/
//Key_Value:Remote Control
extern Roller_State_t Roller_State[3];	//拨轮方向
extern Stick_Vert_t StickL_Vert[3];			//左摇杆竖直方向
extern Stick_Hori_t StickL_Hori[3];			//左摇杆水平方向
extern Stick_Vert_t StickR_Vert[3];			//右摇杆竖直方向
extern Stick_Hori_t StickR_Hori[3];			//右摇杆水平方向
//Key_Value:Keyboard
extern Key_Value_t W[3];
extern Key_Value_t S[3];
extern Key_Value_t D[3];
extern Key_Value_t A[3];
extern Key_Value_t Q[3];
extern Key_Value_t E[3];
extern Key_Value_t R[3];
extern Key_Value_t F[3];
extern Key_Value_t G[3];
extern Key_Value_t Z[3];
extern Key_Value_t X[3];
extern Key_Value_t C[3];
extern Key_Value_t V[3];
extern Key_Value_t B[3];
extern Key_Value_t CTRL[3];
extern Key_Value_t SHIFT[3];
extern Key_Value_t Mouse_Left[3];
extern Key_Value_t Mouse_Right[3];
//鼠标速度
extern float Out_X;
extern float Out_Y;

extern PCMode_t PCMode;
extern SubState_t SubState_Last;
/**Function Declaration**/
//Task
void Task_StateMachine(void * parameters);
//Init
void StateMachine_Init(void);
void StatePara_Init(void);
//StateMachine Update
void StateMachine_Update(void);
void MainState_Update(void);
void SubState_Update(void);
void ControlMode_Update(void);
void PCMode_Update(void);
//Key Value Update
void Key_Update(void);
void Switch_Update(void);
void Mouse_Speed_Filter(void);
void Roller_PC_Update(void);


void Manual_Test(void);





float KalmanFilter(int16_t inData);

//State Read
MainState_t GetMainState(void);
SubState_t GetSubState(void);
ControlMode_t GetControlMode(void);
PCMode_t GetPCMode(void);
#endif
