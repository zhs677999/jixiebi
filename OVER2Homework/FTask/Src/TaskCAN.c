#include "TaskCAN.h"
#include "debug_info.h" //debug使用

CanSend_Type CAN_Tx_Msg;
Debug_Info_t debug_info;
int tcancount=0;



void Task_CAN(void *parameters)
{
  while (1)
  {
    tcancount++;
    // start_time3 = xTaskGetTickCount();
    xQueueReceive(Queue_CANSend, &CAN_Tx_Msg, portMAX_DELAY); // 从CAN队列接收需要发送的数据

    switch (CAN_Tx_Msg.CANx) // 根据数据选定的CAN接口进行数据发送
    {
    case CANSEND_1:
      CANTransmit(&hfdcan1, CAN_Tx_Msg.stdid, CAN_Tx_Msg.Data);
      break;

    case CANSEND_2:
      CANTransmit(&hfdcan2, CAN_Tx_Msg.stdid, CAN_Tx_Msg.Data);
      break;
/*		case CANSEND_3:
      CANTransmit(&hfdcan3, CAN_Tx_Msg.stdid, CAN_Tx_Msg.Data);
      break;*/
    default:
      break;
    }

    // can = xTaskGetTickCount() - start_time3;
  }
}

void CANTransmit(FDCAN_HandleTypeDef *hfdcan, uint32_t std_id, uint8_t aData[])
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


/**
 * @brief  CAN初始化
 * @note   滤波
 * @param  hfdcan 的地址
 * @retval None
 */
void CAN_Fliter_Init(FDCAN_HandleTypeDef *hfdcan)
{
  uint32_t IdType, FliterFIFO, WaterMark;
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
  /*else if (hfdcan == &hfdcan3)
  {
    IdType = FDCAN_STANDARD_ID;
    FliterFIFO = FDCAN_FILTER_TO_RXFIFO0;
    WaterMark = FDCAN_CFG_RX_FIFO0;
  }*/
  else
  {
    return;
  }

  // 配置滤波器
  sFilterConfig.IdType = IdType;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FliterFIFO;
  sFilterConfig.FilterID1 = 0x00; //
  sFilterConfig.FilterID2 = 0x00; //
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
  /* 全局过滤设置 */
  /* 接收到消息ID与标准ID过滤不匹配，不接受 */
  /* 接收到消息ID与扩展ID过滤不匹配，不接受 */
  /* 不接受标准ID远程帧 */
  /* 不接受扩展ID远程帧 */
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
/*  else if (hfdcan == &hfdcan3)
  {
    ActiveITs = FDCAN_IT_RX_FIFO0_NEW_MESSAGE;
  }
	*/
  else
  {
    return;
  }

  CAN_Fliter_Init(hfdcan);
  HAL_FDCAN_Start(hfdcan);
  HAL_FDCAN_ActivateNotification(hfdcan, ActiveITs, 0);
}
