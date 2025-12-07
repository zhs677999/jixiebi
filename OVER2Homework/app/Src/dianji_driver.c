#include "dianji_driver.h"
#include "Taskcan.h"
//#include "can.h"

#include "task_pid.h"


//下面对应整体电机,写发送数据

/*
void MotortypeRegister(motor_t *mot, motor_type type, pid_type *pospid =0, pid_type *speedpid, uint8_t Reductionratio, uint32_t Can_id)
{
    if (mot != NULL)
    {
        mot->id = Can_id;
        mot->type = type;
        mot->Reductionratio = Reductionratio;
        mot->PositionPID = pospid;
        mot->SpeedPID = speedpid;
    }
}*/


//注册上线
void MotortypeRegister(motor_t *mot, motor_type type,  uint8_t Reductionratio, uint32_t Can_id)
{
    if (mot != NULL)
    {
        mot->id = Can_id;
        mot->type = type;
        mot->Reductionratio = Reductionratio;

    }
}

void MotortypeRegisterpid(motor_t *mot,
                       motor_type type,
                       pid_type *pospid,
                       pid_type *speedpid,
                       uint8_t Reductionratio,
                       uint32_t Can_id)
{
    if (mot != NULL)
    {
        mot->id            = Can_id;
        mot->type          = type;
        mot->Reductionratio = Reductionratio;

        mot->PositionPID   = pospid;     
        mot->SpeedPID      = speedpid;   
    }
}
	
motor_t YawB,PitchMid;
motor_t DJIup;
 
pid_type YawB_PositionPid;
pid_type YawB_SpeedPid;

pid_type PitchMid_PositionPid;
pid_type PitchMid_SpeedPid;

pid_type DJIup_PositionPid;
pid_type DJIup_SpeedPid;


int Motor_Define =0; 
void MotionMotor_Init(void)
{
    MotortypeRegisterpid(&YawB, MOTOR_6020,
                         &YawB_PositionPid, &YawB_SpeedPid,
    MotortypeRegisterpid(&PitchMid,   MOTOR_4010,
                         &PitchMid_PositionPid, &PitchMid_SpeedPid,

    // 2) PID
    float yawb_angle_para[3] = { 15.0f, 0.0f, 0.1f };
    float yawb_speed_para[3] = { 100.0f, 0.150f, 10.0f };
    float up_angle_para[3]   = { 0.0f, 0.0f, 0.0f };
    PID_Parameter_Init(YawB.PositionPID,
                       yawb_angle_para,
    PID_Parameter_Init(PitchMid.PositionPID,
    PID_Parameter_Init(YawB.SpeedPID,
                       yawb_speed_para,
    // PitchMid ٶȻ PID
    PID_Parameter_Init(PitchMid.SpeedPID,
    PID_clear(YawB.PositionPID);
    PID_clear(YawB.SpeedPID);
    PID_clear(PitchMid.PositionPID);
    PID_clear(PitchMid.SpeedPID);
    YawB.InitAngle   = 117.0f;
    YawB.TargetAngle = YawB.InitAngle;
    YawB.Targetcirnum = YawB.Realcirnum;
   // PitchMid.InitAngle   = 88.9f;
	 PitchMid.InitAngle=PitchMid.RealAngle;
    PitchMid.TargetAngle = PitchMid.InitAngle;
    PitchMid.Targetcirnum = PitchMid.Realcirnum;
    YawB.TargetAngle = 0;
    PitchMid.TargetAngle   = 0;
    YawB.Targetcirnum = DJIup.Targetcirnum = PitchMid.Targetcirnum = 0;
    PID_Parameter_Init(DJIup.PositionPID,
                       PID_POSITION,
                       up_angle_para,
                       10000.0f,
                       500.0f,
                       0);

   
    PID_Parameter_Init(LKmid.PositionPID,
                       PID_POSITION,
                       lk_angle_para,
                       10000.0f,  // 这里先和 6020 一样，后面根据实际再缩放
                       500.0f,
                       0);

    // -------- 速度环 PID（位置式）---------
    // 输出直接给电流控制（例如 DJI_GM6020_CurrentControl）
    PID_Parameter_Init(DJIdown.SpeedPID,
                       PID_POSITION,
                       down_speed_para,
                       5000.0f,    
                       1000.0f,    
                       0);

    PID_Parameter_Init(DJIup.SpeedPID,
                       PID_POSITION,
                       up_speed_para,
                       5000.0f,    
                       1000.0f,
                       0);

    // LKmid 速度环 PID
    PID_Parameter_Init(LKmid.SpeedPID,
                       PID_POSITION,
                       lk_speed_para,
                       1000.0f,    
                       300.0f,
                       0);

    // 3) 清零 PID 状态
    PID_clear(DJIdown.PositionPID);
    PID_clear(DJIup.PositionPID);
    PID_clear(DJIdown.SpeedPID);
    PID_clear(DJIup.SpeedPID);
    PID_clear(LKmid.PositionPID);
    PID_clear(LKmid.SpeedPID);

    // 4) 初始角度 / 圈数
    DJIdown.InitAngle   = 117.0f;
    DJIdown.TargetAngle = DJIdown.InitAngle;
    DJIdown.Targetcirnum = DJIdown.Realcirnum;

    DJIup.InitAngle   = 29.3f;
    DJIup.TargetAngle = DJIup.InitAngle;
    DJIup.Targetcirnum = DJIup.Realcirnum;

   // LKmid.InitAngle   = 88.9f;
	 LKmid.InitAngle=LKmid.RealAngle;
    LKmid.TargetAngle = LKmid.InitAngle;
    LKmid.Targetcirnum = LKmid.Realcirnum;

    /*
    DJIdown.TargetAngle = 0;
    DJIup.TargetAngle   = 0;
    LKmid.TargetAngle   = 0;
    DJIdown.Targetcirnum = DJIup.Targetcirnum = LKmid.Targetcirnum = 0;
    */

    Motor_Define = 1;
}

//下面对应DJI电机

//处理电机旋转角度
//int8_t DJIMotor_AngleHandle(motor_t *mot) 写到can.c，与其他代码一致



/**
 * @brief  大疆 GM6020 电机电流控制（单电机版，入队）
 * @param  currentA  目标电流(A)，范围建议 -3~3
 * @param  motor_id  电机 ID（1~4，对应电流帧 0x1FE）
 * @param  canx      使用哪路 CAN：CANSEND_1 / CANSEND_2
 */
// 直接传 PID 输出的电流指令码（int16_t），对应out
void DJI_GM6020_CurrentControl(int16_t cmd, uint8_t motor_id, uint8_t canx)
{
    static CanSend_Type CANSend;

    // 选用哪路 CAN
    CANSend.CANx  = canx;
    CANSend.stdid = GM6020_CURRENT_CTRL;  // 0x1FE，对应 ID1~4

    // 先全部置 0
    int16_t i1 = 0, i2 = 0, i3 = 0, i4 = 0;

    // 根据 motor_id 把电流码放到对应位置
    switch (motor_id)
    {
    case 1: i1 = cmd; break;
    case 2: i2 = cmd; break;
    case 3: i3 = cmd; break;
    case 4: i4 = cmd; break;
    default:
        return;
    }

void YawB_AngleSpeedCurrent_Loop(motor_t *mot)
    float Dst_YawBAngle, Rel_YawBAngle;
    Dst_YawBAngle = mot->Targetcirnum * 360.0f + mot->TargetAngle;
    Rel_YawBAngle = mot->Realcirnum   * 360.0f + mot->RealAngle;
                                      Dst_YawBAngle,
                                      Rel_YawBAngle);
// ==================== LK мPitchMid ====================
void PitchMid_AngleSpeedCurrent_Loop(motor_t *mot)
{
    float Dst_PitchMidAngle = mot->Targetcirnum * 360.0f + mot->TargetAngle;
        Dst_PitchMidAngle,           


    DJI_GM6020_CurrentControl(mot->Out, DJI_up_id, CANSEND_2);
}


// ==================== LK 中间电机：LKmid ====================
void LKmid_AngleSpeedCurrent_Loop(motor_t *mot)
{/*
		float Dst_LKmidAngle = mot->Targetcirnum * 360.0f + mot->TargetAngle;
    float Rel_LKmidAngle = mot->Realcirnum   * 360.0f + mot->RealAngle;


    mot->Targetrotationrate = Pid_cal(mot->PositionPID,
                                      Dst_LKmidAngle,
                                      Rel_LKmidAngle);
    LIMIT(mot->Targetrotationrate, -2000, 2000);


    float out_pid = Pid_cal(mot->SpeedPID,
                            mot->Targetrotationrate,
                            mot->Realrotationrate);
    int16_t out_cmd = (int16_t)out_pid;


    const float LK_UP = (-360.0f + 350.0f);
    float err = LK_UP - Rel_LKmidAngle;

    int16_t comp = 0;

    if (err > 10.0f)
    {
        comp = +err;   
    }
   
    else if (err < -10.0f)
    {
        comp = -err;
    }
    else
    {
        comp = 0;     
    }

    out_cmd += comp;

   
    if (out_cmd >  100) out_cmd =  100;
    if (out_cmd < -100) out_cmd = -100;

    mot->Out = -out_cmd;

    
    LK_iqControl(mot->Out, LK_mid_id, CANSEND_1);*/
		float Dst_LKmidAngle = mot->Targetcirnum * 360.0f + mot->TargetAngle;

    /* 直接调用电机内部多环位置控制，转到目标位置 */
    LK_MultiLoop_angleControl_limited(
        100,                     
        Dst_LKmidAngle,           
        LK_mid_id,               
        CANSEND_1,                
        (uint16_t)mot->Reductionratio   
    );
}




/* 达妙电机
void DM_speedpositionControl(uint8_t Motor_ID, uint8_t CAN_ID, float _pos, float _vel)
{
    uint8_t *pbuf, *vbuf;

    _pos = _pos * 12.5f / 720.0f;  //!!! 核心缩放

    pbuf = (uint8_t *)&_pos;
    vbuf = (uint8_t *)&_vel;

    static CanSend_Type CANSend;

    CANSend.CANx = CAN_ID;
    CANSend.stdid = DMJ_STDID + Motor_ID;

    CANSend.Data[0] = *pbuf;
    CANSend.Data[1] = *(pbuf + 1);
    CANSend.Data[2] = *(pbuf + 2);
    CANSend.Data[3] = *(pbuf + 3);
    CANSend.Data[4] = *vbuf;
    CANSend.Data[5] = *(vbuf + 1);
    CANSend.Data[6] = *(vbuf + 2);
    CANSend.Data[7] = *(vbuf + 3);

    xQueueSend(Queue_CANSend, &CANSend, 3);
}
*/

//下面对应LK电机

void LK_MultiLoop_angleControl(int32_t angle_control, uint8_t Motor_ID, uint8_t CAN_ID)//LK电机控制角度
{
    static CanSend_Type CANSend;

    CANSend.CANx = CAN_ID;

    CANSend.stdid = LK_STDID + Motor_ID;

    CANSend.Data[0] = 0xA3;
    CANSend.Data[1] = 0;
    CANSend.Data[2] = 0;
    CANSend.Data[3] = 0;
    CANSend.Data[4] = *(uint8_t *)(&angle_control);
    CANSend.Data[5] = *((uint8_t *)(&angle_control) + 1);
    CANSend.Data[6] = *((uint8_t *)(&angle_control) + 2);
    CANSend.Data[7] = *((uint8_t *)(&angle_control) + 3);

    xQueueSend(Queue_CANSend, &CANSend, 3);
}
void LK_MultiLoop_angleControl_limited(uint16_t max_speed, float angle_control_float, uint8_t Motor_ID, uint8_t CAN_ID, uint16_t reductionratio)
{
    static CanSend_Type CANSend;

    CANSend.CANx = CAN_ID;

    CANSend.stdid = LK_STDID + Motor_ID;

    int32_t angle_control = angle_control_float * 100 * reductionratio;
    int16_t speed_control = max_speed * reductionratio;
    //    int32_t angle_control = angle_control_float*100;
    CANSend.Data[0] = 0xA4;
    CANSend.Data[1] = 0;
    CANSend.Data[2] = *(uint8_t *)(&speed_control);
    CANSend.Data[3] = *((uint8_t *)(&speed_control) + 1);
    CANSend.Data[4] = *(uint8_t *)(&angle_control);
    CANSend.Data[5] = *((uint8_t *)(&angle_control) + 1);
    CANSend.Data[6] = *((uint8_t *)(&angle_control) + 2);
    CANSend.Data[7] = *((uint8_t *)(&angle_control) + 3);

    xQueueSend(Queue_CANSend, &CANSend, 3);
}


void LK_MultiLoop_angleRead(uint8_t Motor_ID, uint8_t CAN_ID)
{
    static CanSend_Type CANSend;

    CANSend.CANx = CAN_ID;

    CANSend.stdid = LK_STDID + Motor_ID;

    CANSend.Data[0] = 0x92;
    CANSend.Data[1] = 0;
    CANSend.Data[2] = 0;
    CANSend.Data[3] = 0;
    CANSend.Data[4] = 0;
    CANSend.Data[5] = 0;
    CANSend.Data[6] = 0;
    CANSend.Data[7] = 0;

    xQueueSend(Queue_CANSend, &CANSend, 3);
}

void LK_iqControl(int32_t iq_control, uint8_t Motor_ID, uint8_t CAN_ID)
{
    static CanSend_Type CANSend;

    CANSend.CANx = CAN_ID;

    CANSend.stdid = LK_STDID + Motor_ID;

    CANSend.Data[0] = 0xA1;
    CANSend.Data[1] = 0;
    CANSend.Data[2] = 0;
    CANSend.Data[3] = 0;
    CANSend.Data[4] = *(uint8_t *)(&iq_control);
    CANSend.Data[5] = *((uint8_t *)(&iq_control) + 1);
    CANSend.Data[6] = 0;
    CANSend.Data[7] = 0;

    xQueueSend(Queue_CANSend, &CANSend, 3);
}
