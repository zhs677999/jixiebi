#ifndef ___CAN_H
#define ___CAN_H

//#include "sysconfig.h"
#include "dianjiDriver.h"




extern motor_t YawB, PitchMid;
extern motor_t DJIup;

//下面的移动到TaskCAN.h 方便调用init
//void CAN_Init(FDCAN_HandleTypeDef *hfdcan);
//void CAN_Fliter_Init(FDCAN_HandleTypeDef *hfdcan);

void DJIMotor_ParaHandle(motor_t *mot, uint8_t adata[]);
int8_t DJIMotor_AngleHandle(motor_t *mot); //DJI电机处理
void LK_Motor_ParaHandle(motor_t *motor, uint8_t aData[]);


void Can1Received_infoHandle(uint32_t stdid,uint8_t adata[]);
void Can2Received_infoHandle(uint32_t stdid,uint8_t adata[]);

#endif
