#include "control_system.h"

RCDecoding_Type Remote_controler = {0};
uint8_t RCBuffer[2][RC_FRAME_LEN + RC_FRAME_LEN_BACK] = {0};
uint8_t UartReceive_Data[UartCommLen] = {0};

motor_t YawB = {0};
motor_t PitchMid = {0};
motor_t DJIdown = {0};
motor_t LKmid = {0};
int Motor_Define = 0;
Debug_Info_t debug_info = {0};

static pid_type yaw_position_pid = {0};
static pid_type yaw_speed_pid = {0};
static pid_type pitch_position_pid = {0};
static pid_type pitch_speed_pid = {0};
static pid_type down_position_pid = {0};
static pid_type down_speed_pid = {0};
static pid_type lk_position_pid = {0};
static pid_type lk_speed_pid = {0};

static uint32_t can1_irq_cnt = 0;
static uint32_t can2_irq_cnt = 0;

float MechanicalAngle2RealAngle(uint16_t Mechanical_Angle)
{
    return Mechanical_Angle / 8191.0f * 360.0f;
}

static float clampf(float value, float min, float max)
{
    return fminf(fmaxf(value, min), max);
}

int float_to_uint(float x, float x_min, float x_max, int bits)
{
    const float span = x_max - x_min;
    if (span <= 0.0f || bits <= 0)
    {
        return 0;
    }

    const float clamped = clampf(x, x_min, x_max);
    const unsigned int scale = (1u << bits) - 1u;
    return (int)((clamped - x_min) * ((float)scale) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    const float span = x_max - x_min;
    if (span <= 0.0f || bits <= 0)
    {
        return x_min;
    }

    const float scale = (float)((1u << bits) - 1u);
    return ((float)x_int) * span / scale + x_min;
}

uint8_t PID_Parameter_Init(pid_type *pid, PID_MODE mode, float Pidpara[], uint32_t MaxOut, uint32_t MaxIout, uint16_t DeadZone)
{
    if (pid == NULL || Pidpara == NULL)
    {
        return 0;
    }

    pid->pidmode = mode;
    pid->kp = Pidpara[0];
    pid->ki = Pidpara[1];
    pid->kd = Pidpara[2];
    pid->MaxOut = MaxOut;
    pid->MaxIout = MaxIout;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->sum_error = 0.0f;
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->Pout = pid->Iout = pid->Dout = pid->Out = 0.0f;
    pid->DeadZone = DeadZone;
    return 1;
}

uint8_t PID_Parameter_Update(pid_type *pid, float Pidpara[])
{
    if (pid == NULL || Pidpara == NULL)
    {
        return 0;
    }

    pid->kp = Pidpara[0];
    pid->ki = Pidpara[1];
    pid->kd = Pidpara[2];
    return 1;
}

float Pid_cal(pid_type *PID, float dst, float real)
{
    if (PID == NULL)
    {
        return 0.0f;
    }

    PID->error[2] = PID->error[1];
    PID->error[1] = PID->error[0];
    PID->dst = dst;
    PID->real = real;
    PID->error[0] = dst - real;

    if (PID->pidmode == PID_POSITION)
    {
        PID->Pout = PID->kp * PID->error[0];
        PID->sum_error += PID->error[0];
        PID->Iout = PID->ki * PID->sum_error;
        PID->Dbuf[2] = PID->Dbuf[1];
        PID->Dbuf[1] = PID->Dbuf[0];
        PID->Dbuf[0] = (PID->error[0] - PID->error[1]);
        PID->Dout = PID->kd * PID->Dbuf[0];
        LIMIT(PID->Iout, (-1 * PID->MaxIout), PID->MaxIout);
        PID->Out = PID->Pout + PID->Iout + PID->Dout;
        LIMIT(PID->Out, (-1 * PID->MaxOut), PID->MaxOut);
    }
    else if (PID->pidmode == PID_DELTA)
    {
        PID->Pout = PID->kp * (PID->error[0] - PID->error[1]);
        PID->Iout = PID->ki * PID->error[0];
        PID->Dbuf[2] = PID->Dbuf[1];
        PID->Dbuf[1] = PID->Dbuf[0];
        PID->Dbuf[0] = (PID->error[0] - 2.0f * PID->error[1] + PID->error[2]);
        PID->Dout = PID->kd * PID->Dbuf[0];
        PID->Out += PID->Pout + PID->Iout + PID->Dout;
        LIMIT(PID->Out, (-1 * PID->MaxOut), PID->MaxOut);
    }

    return PID->Out;
}

uint8_t PID_clear(pid_type *pid)
{
    if (pid == NULL)
    {
        return 0;
    }

    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    pid->sum_error = 0.0f;
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    pid->Out = pid->Pout = pid->Iout = pid->Dout = 0.0f;
    return 1;
}

void MotortypeRegister(motor_t *mot, motor_type type, uint8_t Reductionratio, uint32_t Can_id)
{
    if (mot != NULL)
    {
        mot->id = Can_id;
        mot->type = type;
        mot->Reductionratio = Reductionratio;
    }
}

void MotortypeRegisterpid(motor_t *mot, motor_type type, pid_type *pospid, pid_type *speedpid, uint8_t Reductionratio, uint32_t Can_id)
{
    if (mot != NULL)
    {
        mot->id = Can_id;
        mot->type = type;
        mot->Reductionratio = Reductionratio;
        mot->PositionPID = pospid;
        mot->SpeedPID = speedpid;
    }
}

static void init_pid_defaults(void)
{
    float yaw_angle_para[3] = {15.0f, 0.0f, 0.1f};
    float yaw_speed_para[3] = {100.0f, 0.150f, 10.0f};
    float pitch_angle_para[3] = {20.0f, 0.0f, 0.15f};
    float pitch_speed_para[3] = {80.0f, 0.12f, 8.0f};
    float down_angle_para[3] = {15.0f, 0.0f, 0.05f};
    float down_speed_para[3] = {100.0f, 0.15f, 10.0f};
    float lk_angle_para[3] = {18.0f, 0.0f, 0.05f};
    float lk_speed_para[3] = {60.0f, 0.12f, 6.0f};

    PID_Parameter_Init(&yaw_position_pid, PID_POSITION, yaw_angle_para, 20000.0f, 5000.0f, 0);
    PID_Parameter_Init(&yaw_speed_pid, PID_POSITION, yaw_speed_para, 5000.0f, 1500.0f, 0);
    PID_Parameter_Init(&pitch_position_pid, PID_POSITION, pitch_angle_para, 20000.0f, 5000.0f, 0);
    PID_Parameter_Init(&pitch_speed_pid, PID_POSITION, pitch_speed_para, 5000.0f, 1500.0f, 0);
    PID_Parameter_Init(&down_position_pid, PID_POSITION, down_angle_para, 20000.0f, 5000.0f, 0);
    PID_Parameter_Init(&down_speed_pid, PID_POSITION, down_speed_para, 5000.0f, 1500.0f, 0);
    PID_Parameter_Init(&lk_position_pid, PID_POSITION, lk_angle_para, 10000.0f, 500.0f, 0);
    PID_Parameter_Init(&lk_speed_pid, PID_POSITION, lk_speed_para, 1000.0f, 300.0f, 0);
}

void MotionMotor_Init(void)
{
    init_pid_defaults();

    MotortypeRegisterpid(&YawB, MOTOR_6020, &yaw_position_pid, &yaw_speed_pid, 1, GM6020_FEEDBACK_ID + 1);
    MotortypeRegisterpid(&PitchMid, MOTOR_4010, &pitch_position_pid, &pitch_speed_pid, 1, 0x140 + 1);
    MotortypeRegisterpid(&DJIdown, MOTOR_6020, &down_position_pid, &down_speed_pid, 1, GM6020_FEEDBACK_ID + 2);
    MotortypeRegisterpid(&LKmid, MOTOR_4010, &lk_position_pid, &lk_speed_pid, 6, 0x140 + 2);

    PID_clear(YawB.PositionPID);
    PID_clear(YawB.SpeedPID);
    PID_clear(PitchMid.PositionPID);
    PID_clear(PitchMid.SpeedPID);
    PID_clear(DJIdown.PositionPID);
    PID_clear(DJIdown.SpeedPID);
    PID_clear(LKmid.PositionPID);
    PID_clear(LKmid.SpeedPID);

    YawB.InitAngle = 0.0f;
    PitchMid.InitAngle = 0.0f;
    DJIdown.InitAngle = 0.0f;
    LKmid.InitAngle = 0.0f;

    YawB.TargetAngle = PitchMid.TargetAngle = DJIdown.TargetAngle = LKmid.TargetAngle = 0.0f;
    YawB.Targetcirnum = PitchMid.Targetcirnum = DJIdown.Targetcirnum = LKmid.Targetcirnum = 0;

    Motor_Define = 1;
}

void DJIMotor_ParaHandle(motor_t *mot, uint8_t adata[])
{
    if (mot == NULL || mot->type != MOTOR_6020)
    {
        return;
    }

    mot->FeedbackData.FrameCounter++;
    if (mot->First_Frame == 1)
    {
        mot->FeedbackData.Mechanical_Angle[1] = mot->FeedbackData.Mechanical_Angle[0];
    }

    mot->FeedbackData.Mechanical_Angle[0] = (uint16_t)((adata[0] << 8) | adata[1]);
    if (mot->First_Frame == 0)
    {
        mot->FeedbackData.Mechanical_Angle[1] = mot->FeedbackData.Mechanical_Angle[0];
        mot->First_Frame = 1;
    }

    mot->FeedbackData.RealSpeed = (int16_t)((adata[2] << 8) | adata[3]);
    mot->FeedbackData.Current = (int16_t)((adata[4] << 8) | adata[5]);
    mot->Realrotationrate = mot->FeedbackData.RealSpeed;

    DJIMotor_AngleHandle(mot);
}

int8_t DJIMotor_AngleHandle(motor_t *mot)
{
    if (mot == NULL || mot->type != MOTOR_6020)
    {
        return 0;
    }

    if (mot->FeedbackData.Mechanical_Angle[0] > 6700 && mot->FeedbackData.Mechanical_Angle[1] < 1500)
    {
        mot->Realcirnum -= 1;
        return -1;
    }
    if (mot->FeedbackData.Mechanical_Angle[0] < 1500 && mot->FeedbackData.Mechanical_Angle[1] > 6700)
    {
        mot->Realcirnum += 1;
        return 1;
    }

    float tempAngle = MechanicalAngle2RealAngle(mot->FeedbackData.Mechanical_Angle[0]) / (float)(mot->Reductionratio);
    while (tempAngle >= 360)
    {
        tempAngle -= 360;
    }
    while (tempAngle < 0)
    {
        tempAngle += 360;
    }

    mot->RealAngle = tempAngle;
    return 0;
}

void LK_Motor_ParaHandle(motor_t *motor, uint8_t aData[])
{
    if (motor == NULL)
    {
        return;
    }

    motor->FeedbackData.FrameCounter++;
    uint8_t frame_id = aData[0];
    if (frame_id == 0xA0 || frame_id == 0xA1 || frame_id == 0xA2 || frame_id == 0xA4)
    {
        motor->FeedbackData.Temperature = (int8_t)aData[1];
        motor->FeedbackData.Current = (int16_t)(aData[2] | aData[3] << 8);
        motor->FeedbackData.RealSpeed = ((int16_t)(aData[4] | aData[5] << 8));
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[6] | aData[7] << 8);
        motor->Realrotationrate = motor->FeedbackData.RealSpeed;
    }
    else if (frame_id == 0x90)
    {
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[2] | aData[3] << 8);
    }
    else if (frame_id == 0x92)
    {
        int64_t angle1 = ((int64_t)aData[7] << 24 | (int64_t)aData[6] << 16 | (int64_t)aData[5] << 8 | aData[4]);
        int64_t angle2 = ((int64_t)aData[3] << 24 | (int64_t)aData[2] << 16 | (int64_t)aData[1] << 8);
        int64_t angle_raw = (angle1 << 32) | angle2;
        float angle_deg = angle_raw / 100.0f / 256.0f / motor->Reductionratio;
        int16_t cir = 0;
        while (angle_deg >= 360)
        {
            angle_deg -= 360;
            cir += 1;
        }
        while (angle_deg < 0)
        {
            angle_deg += 360;
            cir -= 1;
        }
        motor->Realcirnum = cir;
        motor->RealAngle = angle_deg;
    }
}

void Can1Received_infoHandle(uint32_t stdid, uint8_t adata[])
{
    uint32_t id = stdid;
    switch (id)
    {
    case (GM6020_FEEDBACK_ID + 3):
        DJIMotor_ParaHandle(&YawB, adata);
        break;
    case (0x140 + 2):
        LK_Motor_ParaHandle(&PitchMid, adata);
        break;
    default:
        break;
    }
}

void Can2Received_infoHandle(uint32_t stdid, uint8_t adata[])
{
    (void)stdid;
    (void)adata;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t aData[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, aData);
    if (hfdcan == &hfdcan1)
    {
        Can1Received_infoHandle(RxHeader.Identifier, aData);
        can1_irq_cnt++;
    }
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    (void)RxFifo0ITs;
    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t aData[8];
    HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, aData);
    Can2Received_infoHandle(RxHeader.Identifier, aData);
    can2_irq_cnt++;
}

static void enqueue_current_command(uint8_t canx, uint8_t motor_id, int16_t cmd)
{
    extern QueueHandle_t Queue_CANSend;
    CanSend_Type CANSend = {0};
    CANSend.CANx = canx;
    CANSend.stdid = GM6020_CURRENT_CTRL;
    int16_t i1 = 0, i2 = 0, i3 = 0, i4 = 0;
    switch (motor_id)
    {
    case 1: i1 = cmd; break;
    case 2: i2 = cmd; break;
    case 3: i3 = cmd; break;
    case 4: i4 = cmd; break;
    default: return;
    }
    CANSend.Data[0] = (uint8_t)(i1 >> 8);
    CANSend.Data[1] = (uint8_t)(i1 & 0xFF);
    CANSend.Data[2] = (uint8_t)(i2 >> 8);
    CANSend.Data[3] = (uint8_t)(i2 & 0xFF);
    CANSend.Data[4] = (uint8_t)(i3 >> 8);
    CANSend.Data[5] = (uint8_t)(i3 & 0xFF);
    CANSend.Data[6] = (uint8_t)(i4 >> 8);
    CANSend.Data[7] = (uint8_t)(i4 & 0xFF);
    xQueueSend(Queue_CANSend, &CANSend, 0);
}

void DJI_GM6020_CurrentControl(int16_t cmd, uint8_t motor_id, uint8_t canx)
{
    enqueue_current_command(canx, motor_id, cmd);
}

void DJI_AngleSpeedCurrent_Loop(motor_t *mot, uint8_t motor_id, uint8_t canx)
{
    if (mot == NULL)
    {
        return;
    }

    float dst_angle = mot->Targetcirnum * 360.0f + mot->TargetAngle;
    float real_angle = mot->Realcirnum * 360.0f + mot->RealAngle;
    float angle_out = Pid_cal(mot->PositionPID, dst_angle, real_angle);
    float speed_out = Pid_cal(mot->SpeedPID, mot->Targetrotationrate + angle_out, mot->Realrotationrate);
    mot->Out = (int16_t)speed_out;
    DJI_GM6020_CurrentControl(mot->Out, motor_id, canx);
}

void YawB_AngleSpeedCurrent_Loop(motor_t *mot)
{
    DJI_AngleSpeedCurrent_Loop(mot, 3, CANSEND_1);
}

void PitchMid_AngleSpeedCurrent_Loop(motor_t *mot)
{
    DJI_AngleSpeedCurrent_Loop(mot, 1, CANSEND_2);
}

void LK_MultiLoop_angleRead(uint8_t Motor_ID, uint8_t CAN_ID)
{
    (void)Motor_ID;
    (void)CAN_ID;
}

void LK_MultiLoop_angleControl_limited(uint16_t max_speed, float angle_control_float, uint8_t Motor_ID, uint8_t CAN_ID, uint16_t reductionratio)
{
    (void)max_speed;
    (void)angle_control_float;
    (void)Motor_ID;
    (void)CAN_ID;
    (void)reductionratio;
}

void LK_iqControl(int32_t iq_control, uint8_t Motor_ID, uint8_t CAN_ID)
{
    (void)iq_control;
    (void)Motor_ID;
    (void)CAN_ID;
}

void LKmid_AngleSpeedCurrent_Loop(motor_t *mot)
{
    DJI_AngleSpeedCurrent_Loop(mot, 2, CANSEND_1);
}

void DJIdown_AngleSpeedCurrent_Loop(motor_t *mot)
{
    DJI_AngleSpeedCurrent_Loop(mot, 4, CANSEND_1);
}

static void reset_rc_buffer(void)
{
    Remote_controler.buffer[0].rc.sw[0] = sw_offset;
    Remote_controler.buffer[0].rc.sw[1] = sw_offset;
    for (int i = 0; i < 5; i++)
    {
        Remote_controler.buffer[0].rc.ch[i] = RC_CH_VALUE_OFFSET;
    }
    Remote_controler.buffer[1] = Remote_controler.buffer[0];
    memset(Remote_controler.mousePresstime, 0, sizeof(Remote_controler.mousePresstime));
    memset(Remote_controler.keyPresstime, 0, sizeof(Remote_controler.keyPresstime));
    Remote_controler.offline_check = 1;
}

uint8_t RcDataUpdate(void)
{
    if (Remote_controler.buffer_index == MEMORYRESET)
    {
        reset_rc_buffer();
        return 0;
    }

    Remote_controler.buffer[Remote_controler.buffer_index].rc.sw[0] = (switch_state)(((RCBuffer[0][5] >> 4) & 0x000C) >> 2);
    Remote_controler.buffer[Remote_controler.buffer_index].rc.sw[1] = (switch_state)((RCBuffer[0][5] >> 4) & 0x0003);

    Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[0] = ((RCBuffer[0][0] | (RCBuffer[0][1] << 8)) & 0x07ff);
    Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[1] = (((RCBuffer[0][1] >> 3) | (RCBuffer[0][2] << 5)) & 0x07ff);
    Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[2] = (((RCBuffer[0][2] >> 6) | (RCBuffer[0][3] << 2) | (RCBuffer[0][4] << 10)) & 0x07ff);
    Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[3] = (((RCBuffer[0][4] >> 1) | (RCBuffer[0][5] << 7)) & 0x07ff);
    Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[4] = ((RCBuffer[0][16] | (RCBuffer[0][17] << 8)) & 0x07ff);

    Remote_controler.buffer[Remote_controler.buffer_index].mouse.x = RCBuffer[0][6] | (RCBuffer[0][7] << 8);
    Remote_controler.buffer[Remote_controler.buffer_index].mouse.y = RCBuffer[0][8] | (RCBuffer[0][9] << 8);
    Remote_controler.buffer[Remote_controler.buffer_index].mouse.z = RCBuffer[0][10] | (RCBuffer[0][11] << 8);
    Remote_controler.buffer[Remote_controler.buffer_index].mouse.press_l = RCBuffer[0][12];
    Remote_controler.buffer[Remote_controler.buffer_index].mouse.press_r = RCBuffer[0][13];
    Remote_controler.buffer[Remote_controler.buffer_index].keyboard.key_bit = RCBuffer[0][14] | (RCBuffer[0][15] << 8);

    Updata_Trigger();
    return 1;
}

void Updata_Trigger(void)
{
    if (Remote_controler.buffer[0].rc.sw[0] != Remote_controler.buffer[1].rc.sw[0])
    {
        Remote_controler.swtrigger[0] = 1;
    }
    if (Remote_controler.buffer[0].rc.sw[1] != Remote_controler.buffer[1].rc.sw[1])
    {
        Remote_controler.swtrigger[1] = 1;
    }
}

float Get_Channel_Value(channel_num channel)
{
    switch (channel)
    {
    case CH_RightHori:
        return (fabsf(Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[0] - RC_CH_VALUE_OFFSET) < 10) ? 0.0f : ((Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[0] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_RightVert:
        return (fabsf(Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[1] - RC_CH_VALUE_OFFSET) < 10) ? 0.0f : ((Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[1] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_LeftHori:
        return (fabsf(Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[2] - RC_CH_VALUE_OFFSET) < 10) ? 0.0f : ((Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[2] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_LeftVert:
        return (fabsf(Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[3] - RC_CH_VALUE_OFFSET) < 10) ? 0.0f : ((Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[3] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_Roll:
        return (fabsf(Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[4] - RC_CH_VALUE_OFFSET) < 10) ? 0.0f : ((Remote_controler.buffer[Remote_controler.buffer_index].rc.ch[4] - RC_CH_VALUE_OFFSET) / 660.0f);
    default:
        return 0.0f;
    }
}

switch_state Get_Switch_Value(rc_switch sw)
{
    switch (sw)
    {
    case Switch_Left:
        return Remote_controler.buffer[Remote_controler.buffer_index].rc.sw[0];
    case Switch_Right:
        return Remote_controler.buffer[Remote_controler.buffer_index].rc.sw[1];
    default:
        return sw_mid;
    }
}

void BSP_Init_RemoteControl(void)
{
    HAL_UART_Receive_DMA(&RC_HUART, RCBuffer[0], (RC_FRAME_LEN + RC_FRAME_LEN_BACK));
    __HAL_UART_ENABLE_IT(&RC_HUART, UART_IT_IDLE);
}

void RC_IRQHandler(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static uint8_t this_time_rx_len = 0;
    if (__HAL_UART_GET_IT_SOURCE(&RC_HUART, UART_IT_IDLE) != RESET)
    {
        __HAL_UART_CLEAR_IDLEFLAG(huart);
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
        this_time_rx_len = (RC_FRAME_LEN + RC_FRAME_LEN_BACK) - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);
        if ((((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_CT) != RESET)
        {
            ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR &= (uint32_t)(~DMA_SxCR_CT);
            Remote_controler.buffer_index = MEMORY1;
        }
        else
        {
            ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR |= (uint32_t)DMA_SxCR_CT;
            Remote_controler.buffer_index = MEMORY0;
        }
        if (this_time_rx_len != RC_FRAME_LEN)
        {
            Remote_controler.buffer_index = MEMORYRESET;
        }
        __HAL_DMA_SET_COUNTER(&hdma_uart5_rx, (RC_FRAME_LEN + RC_FRAME_LEN_BACK));
        __HAL_DMA_ENABLE(&hdma_uart5_rx);
        RcDataUpdate();
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

void Update_motor(void)
{
    (void)Remote_controler;
}

void Angle_Limit(motor_t *Motor)
{
    if (Motor == NULL)
    {
        return;
    }
    while (Motor->TargetAngle >= 360.0f)
    {
        Motor->TargetAngle -= 360.0f;
    }
    while (Motor->TargetAngle < 0.0f)
    {
        Motor->TargetAngle += 360.0f;
    }
}

void Motor_Set_TargetAngle(motor_t *m, float relTargetAngle)
{
    if (m == NULL)
    {
        return;
    }
    m->TargetAngle = relTargetAngle;
    Angle_Limit(m);
}

void Motor_Add_TargetAngle(motor_t *m, float AddAngle)
{
    if (m == NULL)
    {
        return;
    }
    m->TargetAngle += AddAngle;
    Angle_Limit(m);
}

