#include "can.h"
#include "zaxiang.h"
#include "dianji_driver.h"

//下面来自于User_Can.c，下面主要是接收函数

 uint32_t can1_irq_cnt = 0;
 uint32_t can2_irq_cnt = 0;


/**
 * @brief  FIFO0的CAN接收回调函数
 * @note   CAN1
 * @param  对应的CAN句柄
 * @retval None
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
	
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t aData[8];                                                 /*接收数据缓存数组*/
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, aData); /*从邮箱中取回数据*/
  if (hfdcan == &hfdcan1)
  {
    Can1Received_infoHandle(RxHeader.Identifier, aData);
		can1_irq_cnt++;
  }
  /*else if (hfdcan == &hfdcan3)
  {
    Can3Received_infoHandle(RxHeader.Identifier, aData);
  }*/
}

/**
 * @brief  FIFO1的CAN接收回调函数
 * @note   CAN2
 * @param  对应的CAN句柄
 * @retval None
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef RxHeader;
  uint8_t aData[8];                                                 /*接收数据缓存数组*/
  HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &RxHeader, aData); /*从邮箱中取回数据*/
  Can2Received_infoHandle(RxHeader.Identifier, aData);
	can2_irq_cnt++;
}





// 下面是电机的处理函数，处理接受的各种电机数据



void DJIMotor_ParaHandle(motor_t *mot, uint8_t adata[])
{
   if (mot->type == MOTOR_6020)
    {
        mot->FeedbackData.FrameCounter++;

        if (mot->First_Frame == 1)
            mot->FeedbackData.Mechanical_Angle[1] = mot->FeedbackData.Mechanical_Angle[0];

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
    else
        return;
}

int8_t DJIMotor_AngleHandle(motor_t *mot)
{
     if (mot->type == MOTOR_6020)
    {
       
        if (mot->FeedbackData.Mechanical_Angle[0] > 6700 && mot->FeedbackData.Mechanical_Angle[1] < 1500)
        {
            mot->Realcirnum -= 1;
            return -1;
        }
        else if (mot->FeedbackData.Mechanical_Angle[0] < 1500 && mot->FeedbackData.Mechanical_Angle[1] > 6700)
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
    else
    {
        return 0;
    }
}


void LK_Motor_ParaHandle(motor_t *motor, uint8_t aData[])
{
    motor->FeedbackData.FrameCounter++;

    if (aData[0] == 0xA0)
    {
        motor->FeedbackData.Temperature = (int8_t)aData[1];
        motor->FeedbackData.Current = (int16_t)(aData[2] | aData[3] << 8);     // 实际扭矩
        motor->FeedbackData.RealSpeed = ((int16_t)(aData[4] | aData[5] << 8)); // 电机带1:10的减速比，原始数据为dps
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[6] | aData[7] << 8);
        motor->Realrotationrate = motor->FeedbackData.RealSpeed;
    }

    if (aData[0] == 0xA1)
    {
        motor->FeedbackData.Temperature = (int8_t)aData[1];
        motor->FeedbackData.test = (aData[2] | aData[3] << 8) * 66 / 4096;
        motor->FeedbackData.Current = (int16_t)(aData[2] | aData[3] << 8) * 66 / 4096; // 实际扭矩
        motor->FeedbackData.RealSpeed = ((int16_t)(aData[4] | aData[5] << 8));         // 电机带1:10的减速比，原始数据为dps
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[6] | aData[7] << 8);
        motor->Realrotationrate = motor->FeedbackData.RealSpeed;
    }

    if (aData[0] == 0xA2)
    {
        motor->FeedbackData.Temperature = (int8_t)aData[1];
        motor->FeedbackData.Current = (int16_t)(aData[2] | aData[3] << 8);     // 实际扭矩
        motor->FeedbackData.RealSpeed = ((int16_t)(aData[4] | aData[5] << 8)); // 电机带1:10的减速比，原始数据为dps
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[6] | aData[7] << 8);
        motor->Realrotationrate = motor->FeedbackData.RealSpeed;
    }

    if (aData[0] == 0xA4)
    {
        motor->FeedbackData.Temperature = (int8_t)aData[1];
        motor->FeedbackData.Current = (int16_t)(aData[2] | aData[3] << 8);     // 实际扭矩
        motor->FeedbackData.RealSpeed = ((int16_t)(aData[4] | aData[5] << 8)); // 电机带1:10的减速比，原始数据为dps
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[6] | aData[7] << 8);
        motor->Realrotationrate = motor->FeedbackData.RealSpeed;
    }

    if (aData[0] == 0x90)
    {
        motor->FeedbackData.Mechanical_Angle[0] = (uint16_t)(aData[2] | aData[3] << 8);
    }

    if (aData[0] == 0x92)
    {
        int64_t test_angle1 = 0;
        int64_t test_angle2 = 0;
        int64_t test_angleint = 0;
        int16_t test_cirnum = 0;
        float test_angle = 0;
        // 角度回传
        test_angle1 = ((aData[7] << 24 | aData[6] << 16) | aData[5] << 8) | aData[4];
        test_angle2 = ((aData[3] << 24 | aData[2] << 16) | aData[1] << 8) | 0;
        test_angleint = (test_angle1 << 32) | test_angle2;

        test_angle = test_angleint / 100.0f / 256.0f / motor->Reductionratio;

        while (test_angle >= 360)
        {
            test_angle -= 360;
            test_cirnum += 1;
        } // TODO,圈数待改

        while (test_angle < 0)
        {
            test_angle += 360;
            test_cirnum -= 1;
        }

        motor->Realcirnum = test_cirnum;
        motor->RealAngle = test_angle;
    }
    static uint8_t error;
    if (aData[0] == 0x9A)
    {
        error = aData[7];
    }
}






/**
 * @brief  Handle the Received Information though Can1
 * @note   CAN1
 * @param  stdid, adata
 * @retval None
 */
void Can1Received_infoHandle(uint32_t stdid, uint8_t adata[])
{
	uint32_t id = stdid ;
	
  switch (id)
  {
  case (DJI_down_id+0x204)	:
    DJIMotor_ParaHandle(&YawB, adata);
    break;

  case (LK_STDID + LK_mid_id):
	
	//case (0x803):
    //DM_infoHandle(&PitchF_Motor, adata);
		LK_Motor_ParaHandle(&PitchMid, adata);
    break;
/*    原版
  case RollF_Motor_id:
    DM_infoHandle(&RollF_Motor, adata);
    break;*/

  default:
    break;
  }
	
}

/**
 * @brief  Handle the Received Information though Can2
 * @note   CAN2
 * @param  stdid, adata
 * @retval None
 */
void Can2Received_infoHandle(uint32_t stdid, uint8_t adata[])
{
  //uint32_t id = stdid & 0x7FF;
        //id=id-0x204;
  switch (stdid)
  {
  default:
    break;
  }
}
















