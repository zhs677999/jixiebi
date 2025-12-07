/**Include Header Files**/
#include "task_state.h"
#include "uart.h"

/**Macro Definition**/
#define TASK_STATEMACHINE_INTERVAL (15)


extern RCDecoding_Type Remote_controler;
MainState_t MainState;
SubState_t SubState;
ControlMode_t ControlMode;
PCMode_t PCMode;
// Key_Value:Remote Control
Roller_State_t Roller_State[3] = {Roller_Mid, Roller_Mid, Roller_Mid}; // 拨轮方向
Stick_Vert_t StickL_Vert[3] = {Stick_Mid_V, Stick_Mid_V, Stick_Mid_V}; // 左摇杆竖直方向
Stick_Hori_t StickL_Hori[3] = {Stick_Mid_H, Stick_Mid_H, Stick_Mid_H}; // 左摇杆水平方向
Stick_Vert_t StickR_Vert[3] = {Stick_Mid_V, Stick_Mid_V, Stick_Mid_V}; // 右摇杆竖直方向
Stick_Hori_t StickR_Hori[3] = {Stick_Mid_H, Stick_Mid_H, Stick_Mid_H}; // 右摇杆水平方向
// Key_Value:Keyboard
Key_Value_t W[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t S[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t D[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t A[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t Q[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t E[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t R[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t F[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t G[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t Z[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t X[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t C[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t V[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t B[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t CTRL[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t SHIFT[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t Mouse_Left[3] = {UnPressed, UnPressed, UnPressed};
Key_Value_t Mouse_Right[3] = {UnPressed, UnPressed, UnPressed};
// 鼠标速度
float Out_X = 0;
float Out_Y = 0;
float XX = 0;
float YY = 0;
float Mouse_Speed_X[9] = {0};
float Mouse_Speed_Y[9] = {0};

// 堵转检测
SubState_t SubState_Last;  // 用于堵转运行判断
uint8_t SubState_flag = 0; // 用于记录从保护状态切出的次数
uint8_t Reset_flag = 0;    // 用于软重启

/* 姿态解算
Vector_t Real_Arm_Pos;
Vector_t Target_Arm_Pos;
float Trans_Matrix[4][4];
Joint_t Arm_joint[6];

Joint_t target_joint[6];*/
/**Function Declaration**/
/**
 * @brief  Task StateMachine
 * @param  unused
 * @retval void
 * @note   Update StateMachine, Update Action Queue
 */
int stmacount = 0;

void Task_StateMachine(void *parameters)
{

    // Initialize StateMachine Enum
    //StateMachine_Init();
/*
    HAL_GPIO_WritePin(Power_GPIO_Port, Power_Pin, GPIO_PIN_SET);

    ArmMotor_Init();
    Sucker_Init();
    Servant_Motor_Init();
    Motor_Define = 1;*/
    while (1)
    {

        stmacount++;
        // 根据遥控器的数据解算本地变量
        StateMachine_Update();
				Manual_Test();
        /*if (GetSubState() == SubState_Protect) // Protect_flag == 1
        {
            Chassis.Speed_K = 0;
            PCState_Init();
            Sucker_Init();
        }
        else
        {
            if (ControlMode == ControlMode_RC)
            {

                //Motor_Calibrate();
               // RC_Chassis_Control();
                Manual_Test(); // 正常遥控器逻辑
            }

            
        }*/

        //ulTaskNotifyTake(pdTRUE, portMAX_DELAY);我不明白为什么小橘没有对应代码
				vTaskDelay(pdMS_TO_TICKS(TASK_STATEMACHINE_INTERVAL));
    }
}

void Switch_Update(void)
{
    // 每次更新[0]中数据，其余依次存储
    Roller_State[2] = Roller_State[1];
    Roller_State[1] = Roller_State[0];

    StickL_Vert[2] = StickL_Vert[1];
    StickL_Vert[1] = StickL_Vert[0];

    StickL_Hori[2] = StickL_Hori[1];
    StickL_Hori[1] = StickL_Hori[0];

    StickR_Vert[2] = StickR_Vert[1];
    StickR_Vert[1] = StickR_Vert[0];

    StickR_Hori[2] = StickR_Hori[1];
    StickR_Hori[1] = StickR_Hori[0];

    // 拨轮
    if (Get_Channel_Value(CH_Roll) < -0.2f && Get_Channel_Value(CH_Roll) >= -0.6f) //
        Roller_State[0] = Roller_Small_Front;
    else if (Get_Channel_Value(CH_Roll) > 0.2f && Get_Channel_Value(CH_Roll) <= 0.6f) //
        Roller_State[0] = Roller_Small_Back;
    else if (Get_Channel_Value(CH_Roll) < -0.6f)
        Roller_State[0] = Roller_Big_Front;
    else if (Get_Channel_Value(CH_Roll) > 0.6f)
        Roller_State[0] = Roller_Big_Back;
    else
        Roller_State[0] = Roller_Mid;

    // 左摇杆
    if ((Get_Channel_Value(CH_LeftHori)) < 0.2f && (Get_Channel_Value(CH_LeftHori)) > -0.2f) // 避免误拨杆，当前后摇杆的时候限制左右摇杆在+-20%
    {
        if (Get_Channel_Value(CH_LeftVert) < -0.2f && Get_Channel_Value(CH_LeftVert) >= -0.6f)
            StickL_Vert[0] = Stick_Small_Back;
        else if (Get_Channel_Value(CH_LeftVert) > 0.2f && Get_Channel_Value(CH_LeftVert) <= 0.6f)
            StickL_Vert[0] = Stick_Small_Front;
        else if (Get_Channel_Value(CH_LeftVert) < -0.6f)
            StickL_Vert[0] = Stick_Big_Back;
        else if (Get_Channel_Value(CH_LeftVert) > 0.6f)
            StickL_Vert[0] = Stick_Big_Front;
        else
            StickL_Vert[0] = Stick_Mid_V;
    }

    if ((Get_Channel_Value(CH_LeftVert)) < 0.2f && (Get_Channel_Value(CH_LeftVert)) > -0.2f) // 避免误拨杆，当前后摇杆的时候限制左右摇杆在+-20%
    {
        if (Get_Channel_Value(CH_LeftHori) < -0.2f && Get_Channel_Value(CH_LeftHori) >= -0.6f)
            StickL_Hori[0] = Stick_Small_Left;
        else if (Get_Channel_Value(CH_LeftHori) > 0.2f && Get_Channel_Value(CH_LeftHori) <= 0.6f)
            StickL_Hori[0] = Stick_Small_Right;
        else if (Get_Channel_Value(CH_LeftHori) < -0.6f)
            StickL_Hori[0] = Stick_Big_Left;
        else if (Get_Channel_Value(CH_LeftHori) > 0.6f)
            StickL_Hori[0] = Stick_Big_Right;
        else
            StickL_Hori[0] = Stick_Mid_H;
    }

    // 右摇杆
    if ((Get_Channel_Value(CH_RightHori)) < 0.2f && (Get_Channel_Value(CH_RightHori)) > -0.2f) // 避免误拨杆，当前后摇杆的时候限制左右摇杆在+-20%
    {
        if (Get_Channel_Value(CH_RightVert) < -0.2f && Get_Channel_Value(CH_RightVert) >= -0.6f)
            StickR_Vert[0] = Stick_Small_Back;
        else if (Get_Channel_Value(CH_RightVert) > 0.2f && Get_Channel_Value(CH_RightVert) <= 0.6f)
            StickR_Vert[0] = Stick_Small_Front;
        else if (Get_Channel_Value(CH_RightVert) < -0.6f)
            StickR_Vert[0] = Stick_Big_Back;
        else if (Get_Channel_Value(CH_RightVert) > 0.6f)
            StickR_Vert[0] = Stick_Big_Front;
        else
            StickR_Vert[0] = Stick_Mid_V;
    }

    if ((Get_Channel_Value(CH_RightVert)) < 0.2f && (Get_Channel_Value(CH_RightVert)) > -0.2f) // 避免误拨杆，当前后摇杆的时候限制左右摇杆在+-20%
    {
        if (Get_Channel_Value(CH_RightHori) < -0.2f && Get_Channel_Value(CH_RightHori) >= -0.6f)
            StickR_Hori[0] = Stick_Small_Left;
        else if (Get_Channel_Value(CH_RightHori) > 0.2f && Get_Channel_Value(CH_RightHori) <= 0.6f)
            StickR_Hori[0] = Stick_Small_Right;
        else if (Get_Channel_Value(CH_RightHori) < -0.6f)
            StickR_Hori[0] = Stick_Big_Left;
        else if (Get_Channel_Value(CH_RightHori) > 0.6f)
            StickR_Hori[0] = Stick_Big_Right;
        else
            StickR_Hori[0] = Stick_Mid_H;
    }
}

/**
 * @brief  StateMachine Init
 * @note
 * @param  None
 * @retval Initialize MainState, SubState, ControlMode, PCMode
 */
void StateMachine_Init(void)
{
    MainState = MainState_Debug;
    SubState = SubState_Protect;

    ControlMode = ControlMode_RC;
    PCMode = PCMode_Switch;
}

void StatePara_Init(void)
{
    //		Sucker_State = Sucker_Off;
}

/**
 * @brief  StateMachine Update
 * @note
 * @param  None
 * @retval 注意函数之间调用顺序！！！确保实时响应
 */
void StateMachine_Update(void)
{
    // Update State
    MainState_Update();
    SubState_Update();
    //ControlMode_Update();

    // Update Key_Value
    //if (ControlMode == ControlMode_RC)
    {
        Switch_Update();
    }

    /*if (ControlMode == ControlMode_PC || 1)
    {
        Key_Update();
        Mouse_Speed_Filter();
        // Roller_PC_Update();
        Switch_Update();
        // XX = KalmanFilter(Out_X);
        // YY = KalmanFilter(Out_Y);
    }*/
}

uint8_t CleanSwTrigger(rc_switch sw)
{
    if (sw >= swcounter)
    {
        return 0;
    }
    Remote_controler.swtrigger[sw] = 0;
    return 1;
}


/**
 * @brief  Update MainState
 * @note   According to Remote Control
 * @param  None
 * @retval
 */
void MainState_Update(void)
{
    // 遥控器主状态更新
    switch (Get_Switch_Value(Switch_Left))
    {
    case sw_up:
        MainState = MainState_Debug;
        CleanSwTrigger(Switch_Left);
        CleanSwTrigger(Switch_Right);
        break;

    case sw_mid:
        MainState = MainState_Vision;
        CleanSwTrigger(Switch_Left);
        CleanSwTrigger(Switch_Right);
        break;

    case sw_down:
        MainState = MainState_Extend;
        CleanSwTrigger(Switch_Left);
        CleanSwTrigger(Switch_Right);
        break;

    default:
        MainState = MainState_Debug;
        break;
    }
}

/**
 * @brief  Update SubState
 * @note   According to Remote Control and MainState
 * @param  None
 * @retval
 */
void SubState_Update(void)
{
    SubState_Last = SubState;

    switch (MainState)
    {
    case MainState_Debug:
        if (Get_Switch_Value(Switch_Right) == sw_up) // 双拨上位保护
            SubState = SubState_Protect;
        else if (Get_Switch_Value(Switch_Right) == sw_mid)
            SubState = SubState_Test;
        else if (Get_Switch_Value(Switch_Right) == sw_down) // 左上右下为键鼠控制
            SubState = SubState_PC;

        break;

    case MainState_Vision:
        if (Get_Switch_Value(Switch_Right) == sw_up)
            SubState = SubState_Big_Joint;
        else if (Get_Switch_Value(Switch_Right) == sw_mid)
            SubState = SubState_Small_Joint;
        else if (Get_Switch_Value(Switch_Right) == sw_down)
            SubState = SubState_XYZ;

        break;

    case MainState_Extend:
        if (Get_Switch_Value(Switch_Right) == sw_up) // 左下右上
            SubState = SubState_Gimbal;
        else if (Get_Switch_Value(Switch_Right) == sw_mid)
            SubState = SubState_Branch;
        else if (Get_Switch_Value(Switch_Right) == sw_down)
            SubState = SubState_DIY;

        break;

    default:
        break;
    }
/*
    // 对从状态变化情况进行记录
    if (SubState_Last == SubState_Protect && SubState != SubState_Protect)
    {
        SubState_flag++;

        if (SubState_flag == 1)
        {
            Motor_Calibrate_Start();
        }
    }

    if (SubState_Last != SubState_Protect && SubState == SubState_Protect)
    {
    }

 

    if (SubState == SubState_Protect)
    {
        Motor_Protect();
    }

    if (SubState != SubState_Protect)
    {
    }*/
}


static inline void Add_YawB_Target(float add_deg)
{
    Motor_Add_TargetAngle(&YawB, add_deg);
}


static inline void Add_PitchMid_Target(float add_deg)
{
    Motor_Add_TargetAngle(&PitchMid, add_deg);
}

    // ҡǰ/   YawB
        Add_YawB_Target(step);
        Add_YawB_Target(-step);
    // ҡǰ/   PitchMid
        Add_PitchMid_Target(step);
        Add_PitchMid_Target(-step);
int ctr_down=0;

void Manual_Test(void)
{
    // 每次遥杆触发时增加/减少的角度（单位：度）
    const float step = 0.5f;

    // 左摇杆前/后 → 控制 DJIdown
    if (StickL_Vert[0] == Stick_Big_Front && StickL_Vert[1] == Stick_Big_Front)
    {
        Add_DJIdown_Target(step);
			ctr_down++;
    }
    if (StickL_Vert[0] == Stick_Big_Back && StickL_Vert[1] == Stick_Big_Back)
    {
        Add_DJIdown_Target(-step);
			ctr_down++;
    }

    // 右摇杆前/后 → 控制 LKmid
    if (StickR_Vert[0] == Stick_Big_Front && StickR_Vert[1] == Stick_Big_Front)
    {
        Add_LKmid_Target(step);
    }
    if (StickR_Vert[0] == Stick_Big_Back && StickR_Vert[1] == Stick_Big_Back)
    {
        Add_LKmid_Target(-step);
    }

}

