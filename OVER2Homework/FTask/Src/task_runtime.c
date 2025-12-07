#include "task_runtime.h"

TaskHandle_t CAN_TASK_Handle = NULL;
TaskHandle_t MOTORCONTROL_TASK_Handle = NULL;
TaskHandle_t STATEMACHINE_TASK_Handle = NULL;
QueueHandle_t Queue_CANSend = NULL;

static FDCAN_TxHeaderTypeDef build_tx_header(uint32_t std_id)
{
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = std_id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;
    return TxHeader;
}

void Task_CAN(void *parameters)
{
    (void)parameters;
    CanSend_Type CAN_Tx_Msg = {0};
    while (1)
    {
        xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY);
        switch (CAN_Tx_Msg.CANx)
        {
        case CANSEND_1:
            CANTransmit(&hfdcan1, CAN_Tx_Msg.stdid, CAN_Tx_Msg.Data);
            break;
        case CANSEND_2:
            CANTransmit(&hfdcan2, CAN_Tx_Msg.stdid, CAN_Tx_Msg.Data);
            break;
        case CANSEND_3:
            break;
        default:
            break;
        }
    }
}

void CANTransmit(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, uint8_t aData[])
{
    FDCAN_TxHeaderTypeDef TxHeader = build_tx_header(std_id);
    HAL_StatusTypeDef status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, aData);
    if (status == HAL_OK)
    {
        debug_info.can_tx_count++;
    }
    else
    {
        debug_info.last_error = status;
        HAL_FDCAN_Stop(hfdcan);
        HAL_FDCAN_Start(hfdcan);
    }
}

void CAN_Fliter_Init(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t IdType;
    uint32_t FliterFIFO;
    uint32_t WaterMark;
    FDCAN_FilterTypeDef sFilterConfig;

    if (hfdcan == &hfdcan1)
    {
        IdType = FDCAN_STANDARD_ID;
        FliterFIFO = FDCAN_FILTER_TO_RXFIFO0;
        WaterMark = FDCAN_CFG_RX_FIFO0;
    }
    else if (hfdcan == &hfdcan2)
    {
        IdType = FDCAN_STANDARD_ID;
        FliterFIFO = FDCAN_FILTER_TO_RXFIFO1;
        WaterMark = FDCAN_CFG_RX_FIFO1;
    }
    else
    {
        return;
    }

    sFilterConfig.IdType = IdType;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FliterFIFO;
    sFilterConfig.FilterID1 = 0x00;
    sFilterConfig.FilterID2 = 0x00;
    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
    HAL_FDCAN_ConfigFifoWatermark(hfdcan, WaterMark, 1);
}

void CAN_Init(FDCAN_HandleTypeDef *hfdcan)
{
    uint32_t ActiveITs;
    if (hfdcan == &hfdcan1)
    {
        ActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
    }
    else if (hfdcan == &hfdcan2)
    {
        ActiveITs = FDCAN_IT_RX_FIFO1_NEW_MESSAGE;
    }
    else
    {
        return;
    }

    CAN_Fliter_Init(hfdcan);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, ActiveITs, 0);
}

static void ensure_motor_target_safe(motor_t *mot)
{
    if (mot == NULL)
    {
        return;
    }
    mot->TargetAngle = mot->RealAngle;
    mot->Targetcirnum = mot->Realcirnum;
}

void Motor_Stop(void)
{
    CanSend_Type CANSend = {0};
    CANSend.CANx = CANSEND_1;
    CANSend.stdid = GM6020_CURRENT_CTRL;
    memset(CANSend.Data, 0, sizeof(CANSend.Data));
    xQueueSend(Queue_CANSend, &CANSend, 3);

    CANSend.CANx = CANSEND_2;
    xQueueSend(Queue_CANSend, &CANSend, 3);
    LK_iqControl(0, 2, CANSEND_1);
}

void Motor_Protect(void)
{
    Motor_Define = 0;
    ensure_motor_target_safe(&DJIdown);
    ensure_motor_target_safe(&LKmid);
}

void Motor_Control(void)
{
    YawB_AngleSpeedCurrent_Loop(&YawB);
    PitchMid_AngleSpeedCurrent_Loop(&PitchMid);
}

void Task_MotorControl(void *parameters)
{
    (void)parameters;
    TickType_t xLastWakeUpTime = xTaskGetTickCount();
    MotionMotor_Init();
    uint8_t was_running = 0;
    while (1)
    {
        if (Motor_Define == 1 && Get_Switch_Value(Switch_Right) == sw_mid)
        {
            Motor_Control();
            was_running = 1;
        }
        else if (was_running)
        {
            Motor_Stop();
            Motor_Protect();
            was_running = 0;
        }
        vTaskDelayUntil(&xLastWakeUpTime, TASK_MOTORCONTROL_INTERVAL);
    }
}

static Roller_State_t Roller_State[3] = {Roller_Mid, Roller_Mid, Roller_Mid};
static Stick_Vert_t StickL_Vert[3] = {Stick_Mid_V, Stick_Mid_V, Stick_Mid_V};
static Stick_Hori_t StickL_Hori[3] = {Stick_Mid_H, Stick_Mid_H, Stick_Mid_H};
static Stick_Vert_t StickR_Vert[3] = {Stick_Mid_V, Stick_Mid_V, Stick_Mid_V};
static Stick_Hori_t StickR_Hori[3] = {Stick_Mid_H, Stick_Mid_H, Stick_Mid_H};

void Switch_Update(void)
{
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

    float roll = Get_Channel_Value(CH_Roll);
    if (roll < -0.6f)
        Roller_State[0] = Roller_Big_Front;
    else if (roll > 0.6f)
        Roller_State[0] = Roller_Big_Back;
    else if (roll < -0.2f)
        Roller_State[0] = Roller_Small_Front;
    else if (roll > 0.2f)
        Roller_State[0] = Roller_Small_Back;
    else
        Roller_State[0] = Roller_Mid;

    float left_vert = Get_Channel_Value(CH_LeftVert);
    float left_hori = Get_Channel_Value(CH_LeftHori);
    if (fabsf(left_hori) < 0.2f)
    {
        if (left_vert < -0.6f)
            StickL_Vert[0] = Stick_Big_Back;
        else if (left_vert > 0.6f)
            StickL_Vert[0] = Stick_Big_Front;
        else if (left_vert < -0.2f)
            StickL_Vert[0] = Stick_Small_Back;
        else if (left_vert > 0.2f)
            StickL_Vert[0] = Stick_Small_Front;
        else
            StickL_Vert[0] = Stick_Mid_V;
    }

    if (fabsf(left_vert) < 0.2f)
    {
        if (left_hori < -0.6f)
            StickL_Hori[0] = Stick_Big_Left;
        else if (left_hori > 0.6f)
            StickL_Hori[0] = Stick_Big_Right;
        else if (left_hori < -0.2f)
            StickL_Hori[0] = Stick_Small_Left;
        else if (left_hori > 0.2f)
            StickL_Hori[0] = Stick_Small_Right;
        else
            StickL_Hori[0] = Stick_Mid_H;
    }

    float right_vert = Get_Channel_Value(CH_RightVert);
    float right_hori = Get_Channel_Value(CH_RightHori);
    if (fabsf(right_hori) < 0.2f)
    {
        if (right_vert < -0.6f)
            StickR_Vert[0] = Stick_Big_Back;
        else if (right_vert > 0.6f)
            StickR_Vert[0] = Stick_Big_Front;
        else if (right_vert < -0.2f)
            StickR_Vert[0] = Stick_Small_Back;
        else if (right_vert > 0.2f)
            StickR_Vert[0] = Stick_Small_Front;
        else
            StickR_Vert[0] = Stick_Mid_V;
    }

    if (fabsf(right_vert) < 0.2f)
    {
        if (right_hori < -0.6f)
            StickR_Hori[0] = Stick_Big_Left;
        else if (right_hori > 0.6f)
            StickR_Hori[0] = Stick_Big_Right;
        else if (right_hori < -0.2f)
            StickR_Hori[0] = Stick_Small_Left;
        else if (right_hori > 0.2f)
            StickR_Hori[0] = Stick_Small_Right;
        else
            StickR_Hori[0] = Stick_Mid_H;
    }
}

void Task_StateMachine(void *parameters)
{
    (void)parameters;
    while (1)
    {
        Switch_Update();
        vTaskDelay(pdMS_TO_TICKS(TASK_STATEMACHINE_INTERVAL));
    }
}

void TaskInit(void const *parameters)
{
    (void)parameters;
    taskENTER_CRITICAL();
    Queue_CANSend = xQueueCreate(30, sizeof(CanSend_Type));
    BSP_Init_RemoteControl();
    CAN_Init(&hfdcan1);
    CAN_Init(&hfdcan2);

    xTaskCreate(Task_CAN, "Task_CAN", 512, NULL, 14, &CAN_TASK_Handle);
    xTaskCreate(Task_MotorControl, "Task_MotorControl", 512, NULL, 12, &MOTORCONTROL_TASK_Handle);
    xTaskCreate(Task_StateMachine, "Task_StateMachine", 256, NULL, 13, &STATEMACHINE_TASK_Handle);
    taskEXIT_CRITICAL();
    vTaskDelete(NULL);
}

