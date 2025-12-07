#include "uart.h"


int update_angle_succeed=0;
int update_angle_succeed_getfunc=0;
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

RCDecoding_Type Remote_controler;
uint8_t RCBuffer[2][RC_FRAME_LEN + RC_FRAME_LEN_BACK];
uint8_t UartReceive_Data[UartCommLen];

 extern motor_t YawB,PitchMid;

/*Roller_State_t Roller_State[3] = {Roller_Mid, Roller_Mid, Roller_Mid}; // 拨轮方向
Stick_Vert_t StickL_Vert[3] = {Stick_Mid_V, Stick_Mid_V, Stick_Mid_V}; // 左摇杆竖直方向
Stick_Hori_t StickL_Hori[3] = {Stick_Mid_H, Stick_Mid_H, Stick_Mid_H}; // 左摇杆水平方向
Stick_Vert_t StickR_Vert[3] = {Stick_Mid_V, Stick_Mid_V, Stick_Mid_V}; // 右摇杆竖直方向
Stick_Hori_t StickR_Hori[3] = {Stick_Mid_H, Stick_Mid_H, Stick_Mid_H}; // 右摇杆水平方向
*/

uint8_t GetRcIndex(void)
{
    return Remote_controler.buffer_index;
}


uint8_t RcDataUpdate(void)
{
    if (GetRcIndex() == MEMORYRESET)
    {
        Remote_controler.buffer[0].rc.sw[0] = sw_offset;
        Remote_controler.buffer[0].rc.sw[1] = sw_offset;

        Remote_controler.buffer[0].rc.ch[0] = RC_CH_VALUE_OFFSET;
        Remote_controler.buffer[0].rc.ch[1] = RC_CH_VALUE_OFFSET;
        Remote_controler.buffer[0].rc.ch[2] = RC_CH_VALUE_OFFSET;
        Remote_controler.buffer[0].rc.ch[3] = RC_CH_VALUE_OFFSET;
        Remote_controler.buffer[0].rc.ch[4] = RC_CH_VALUE_OFFSET;



        Remote_controler.buffer[1] = Remote_controler.buffer[0];

        Remote_controler.mousePresstime[0] = Remote_controler.mousePresstime[1] = 0;
        Remote_controler.keyPresstime[0] = Remote_controler.keyPresstime[1] = Remote_controler.keyPresstime[2] =
            Remote_controler.keyPresstime[3] = Remote_controler.keyPresstime[4] = Remote_controler.keyPresstime[5] =
                Remote_controler.keyPresstime[6] = Remote_controler.keyPresstime[7] = Remote_controler.keyPresstime[8] =
                    Remote_controler.keyPresstime[9] = Remote_controler.keyPresstime[10] = Remote_controler.keyPresstime[11] =
                        Remote_controler.keyPresstime[12] = Remote_controler.keyPresstime[13] = Remote_controler.keyPresstime[14] =
                            Remote_controler.keyPresstime[15] = 0;
        Remote_controler.offline_check = 1;
        return 0;
    }
    else
    {
        Remote_controler.buffer[GetRcIndex()].rc.sw[0] = (switch_state)(((RCBuffer[0][5] >> 4) & 0x000C) >> 2); //!< Switch left
        Remote_controler.buffer[GetRcIndex()].rc.sw[1] = (switch_state)((RCBuffer[0][5] >> 4) & 0x0003);        //!< Switch right

        Remote_controler.buffer[GetRcIndex()].rc.ch[0] = ((RCBuffer[0][0] | (RCBuffer[0][1] << 8)) & 0x07ff);        //!< Channel 0
        Remote_controler.buffer[GetRcIndex()].rc.ch[1] = (((RCBuffer[0][1] >> 3) | (RCBuffer[0][2] << 5)) & 0x07ff); //!< Channel 1
        Remote_controler.buffer[GetRcIndex()].rc.ch[2] = (((RCBuffer[0][2] >> 6) | (RCBuffer[0][3] << 2) | (RCBuffer[0][4] << 10)) & 0x07ff);
        Remote_controler.buffer[GetRcIndex()].rc.ch[3] = (((RCBuffer[0][4] >> 1) | (RCBuffer[0][5] << 7)) & 0x07ff); //!< Channel 3
        Remote_controler.buffer[GetRcIndex()].rc.ch[4] = ((RCBuffer[0][16] | (RCBuffer[0][17] << 8)) & 0x07ff);      //!< Channel 4

        Remote_controler.buffer[GetRcIndex()].mouse.x = RCBuffer[0][6] | (RCBuffer[0][7] << 8);            //!< Mouse X axis
        Remote_controler.buffer[GetRcIndex()].mouse.y = RCBuffer[0][8] | (RCBuffer[0][9] << 8);            //!< Mouse Y axis
        Remote_controler.buffer[GetRcIndex()].mouse.z = RCBuffer[0][10] | (RCBuffer[0][11] << 8);          //!< Mouse Z axis
        Remote_controler.buffer[GetRcIndex()].mouse.press_l = RCBuffer[0][12];                             //!< Mouse Left Is Press ?
        Remote_controler.buffer[GetRcIndex()].mouse.press_r = RCBuffer[0][13];                             //!< Mouse Right Is Press ?
        Remote_controler.buffer[GetRcIndex()].keyboard.key_bit = RCBuffer[0][14] | (RCBuffer[0][15] << 8); //!< KeyBoard value
    }
   
    Updata_Trigger();
		//Update_motor();
    return 1;
}





//核心：读取数据的代码，其他函数只要读就行
float Get_Channel_Value(channel_num channel)
{
    switch (channel)
    {
    case CH_RightHori:
        if (abs(Remote_controler.buffer[GetRcIndex()].rc.ch[0] - RC_CH_VALUE_OFFSET) < 10)
            return 0;
        else
            return ((Remote_controler.buffer[GetRcIndex()].rc.ch[0] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_RightVert:
        if (abs((Remote_controler.buffer[GetRcIndex()].rc.ch[1] - RC_CH_VALUE_OFFSET)) < 10)
            return 0;
        else
            return ((Remote_controler.buffer[GetRcIndex()].rc.ch[1] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_LeftHori:
        if (abs((Remote_controler.buffer[GetRcIndex()].rc.ch[2] - RC_CH_VALUE_OFFSET)) < 10)
            return 0;
        else
            return ((Remote_controler.buffer[GetRcIndex()].rc.ch[2] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_LeftVert:
        if (abs((Remote_controler.buffer[GetRcIndex()].rc.ch[3] - RC_CH_VALUE_OFFSET)) < 10)
            return 0;
        else
            return ((Remote_controler.buffer[GetRcIndex()].rc.ch[3] - RC_CH_VALUE_OFFSET) / 660.0f);
    case CH_Roll:
        if (abs((Remote_controler.buffer[GetRcIndex()].rc.ch[4] - RC_CH_VALUE_OFFSET)) < 10)
            return 0;
        else
            return ((Remote_controler.buffer[GetRcIndex()].rc.ch[4] - RC_CH_VALUE_OFFSET) / 660.0f);
    default:
        return RC_CH_VALUE_OFFSET;
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


void RC_IRQHandler(UART_HandleTypeDef *huart)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE; /*通知值为二进制信号量，用这个通知会使通知只进行一次*/

    static uint8_t this_time_rx_len = 0; /*本次接收长度*/
    if (__HAL_UART_GET_IT_SOURCE(&huart5, UART_IT_IDLE) != RESET)
    {
        /*清除空闲中断标志位*/


        __HAL_UART_CLEAR_IDLEFLAG(huart);

        /*关闭DMA接收*/
        __HAL_DMA_DISABLE(&hdma_uart5_rx);

        /*计算本次帧长度*/
        this_time_rx_len = (RC_FRAME_LEN + RC_FRAME_LEN_BACK) - __HAL_DMA_GET_COUNTER(&hdma_uart5_rx);

        /*记录本次的DMA内存*/
        if ((((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR & DMA_SxCR_CT) != RESET)
        {
            /* Current memory buffer used is Memory 1 */
            ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR &= (uint32_t)(~DMA_SxCR_CT);
            Remote_controler.buffer_index = MEMORY1;
        }
        else
        {
            /* Current memory buffer used is Memory 0 */
            ((DMA_Stream_TypeDef *)hdma_uart5_rx.Instance)->CR |= (uint32_t)DMA_SxCR_CT;
            Remote_controler.buffer_index = MEMORY0;
        }

        /*如果本次帧长度与RC帧长度不等，重启遥控接收内存*/
        if (this_time_rx_len != RC_FRAME_LEN)
            Remote_controler.buffer_index = MEMORYRESET;

        /*设定DMA发送的长度*/
        __HAL_DMA_SET_COUNTER(&hdma_uart5_rx, (RC_FRAME_LEN + RC_FRAME_LEN_BACK));

        /*重启DMA接收*/
        __HAL_DMA_ENABLE(&hdma_uart5_rx);

        RcDataUpdate();

        // 发送任务通知给遥控任务
/*
		我认为这是状态机
        vTaskNotifyGiveFromISR(STATEMACHINE_TASK_Handle, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken)
            change_count++;
*/
        /*强制FreeRTOS任务切换*/
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
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

    if (Remote_controler.buffer[0].mouse.press_l != Remote_controler.buffer[1].mouse.press_l)
    {
        Remote_controler.mousetrigger[0] = 1;
    }
    if (Remote_controler.buffer[0].mouse.press_r != Remote_controler.buffer[1].mouse.press_r)
    {
        Remote_controler.mousetrigger[1] = 1;
    }
    Remote_controler.keytrigger = (Remote_controler.buffer[0].keyboard.key_bit) ^ (Remote_controler.buffer[1].keyboard.key_bit);
}

void BSP_Init_RemoteControl(void)
{
    Remote_controler.framecounter = 0;           /*帧率计数*/
    Remote_controler.offline_check = 0;          /*离线计数*/
    Remote_controler.buffer_index = MEMORYRESET; /*串口缓存区默认值*/
    SET_BIT(RC_HUART.Instance->CR3, USART_CR3_DMAR);
    HAL_DMAEx_MultiBufferStart(RC_Huart.hdmarx, (uint32_t)&(RC_HUART.Instance->RDR), (uint32_t)&RCBuffer[0][0], (uint32_t)&RCBuffer[1][0], (RC_FRAME_LEN + RC_FRAME_LEN_BACK));
    __HAL_UART_ENABLE_IT(&RC_HUART, UART_IT_IDLE);
	
}

void Angle_Limit(motor_t *Motor)
{
	int16_t temp_Targetcirnum = 0;

	while (Motor->TargetAngle >= 360)
	{
		Motor->TargetAngle -= 360;
		temp_Targetcirnum += 1;
	}

	while (Motor->TargetAngle < 0)
	{
		Motor->TargetAngle += 360;
		temp_Targetcirnum -= 1;
	}

	Motor->Targetcirnum = temp_Targetcirnum;

	return;
}

void Motor_Set_TargetAngle(motor_t* m, float relTargetAngle)
{
    // 目标绝对角度 = InitAngle + 相对角度命令
    m->TargetAngle = m->InitAngle + relTargetAngle;
	if (fabsf(relTargetAngle-( m->TargetAngle + m->Targetcirnum*360.0f - m->InitAngle))>=2.0f)
        m->SpeedPID->sum_error = 0;
    // 保留原来逻辑：对角度做限幅 / 度数归一化 / 更新 Targetcirnum / TargetMechAngle 等
    Angle_Limit(m);
}

// 增量式：在当前目标角度基础上增加 AddAngle（考虑多圈）
void Motor_Add_TargetAngle(motor_t* m, float AddAngle)
{
    float total_target = m->TargetAngle + m->Targetcirnum * 360.0f;
    float now_rel      = total_target - m->InitAngle;
    float new_rel      = now_rel + AddAngle;

    Motor_Set_TargetAngle(m, new_rel);
}


//懒得再开一个，直接开一个设置电机目标值

void Motor_Update_Target_FromCycle(motor_t *m, float angle_cycle)
{
    if (m == NULL)
        return;

    // 1. 第一次使用时，用当前实际角度作为初始目标，防止一上来乱跳
    if (m->First_Frame == 0)
    {
        float total_real = m->RealAngle + m->Realcirnum * 360.0f;
        int32_t cir      = (int32_t)floorf(total_real / 360.0f);
        float   in_cycle = total_real - (float)cir * 360.0f;
        if (in_cycle < 0.0f)
        {
            in_cycle += 360.0f;
            cir -= 1;
        }

        m->Targetcirnum = (int16_t)cir;
        m->TargetAngle  = in_cycle;
        m->First_Frame  = 1;
    }

    // 2. 上一次的“总目标角度（含多圈）”
    float prev_total = m->TargetAngle + m->Targetcirnum * 360.0f;

    // 2. 读取归一化的摇杆值（Get_Channel_Value 返回约 -1.0 ~ +1.0）
    //float ch_down = Get_Channel_Value(CH_LeftVert);   // 左摇杆上下 -> DJIdown
		ch_down = Get_Channel_Value(CH_LeftVert); 
    float ch_mid  = Get_Channel_Value(CH_LeftHori);   // 左摇杆左右 -> LKmid

    // 3. 映射到 0 ~ 360° 之间
    //    -1 对应 0°，0 对应 180°，+1 对应 360°
    float down_cycle = (ch_down + 1.0f) * 180.0f;   // 0 ~ 360
    float mid_cycle  = (ch_mid  + 1.0f) * 180.0f;   // 0 ~ 360

    // 4. 更新两个电机的 TargetAngle / Targetcirnum（处理多圈）
    Motor_Update_Target_FromCycle(&DJIdown, down_cycle);
                update_angle_succeed_getfunc++;
    Motor_Update_Target_FromCycle(&LKmid,   mid_cycle);

}
*/
