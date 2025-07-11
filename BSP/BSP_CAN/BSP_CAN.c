#include "BSP_CAN.h"

CAN_RxHeaderTypeDef RxHeader;
MotorData_t MotorData;
HAL_StatusTypeDef BSP_CAN_Filter_Init(CAN_HandleTypeDef *hhcan)
{
    CAN_FilterTypeDef can_filt=
    {
        .FilterActivation = CAN_FILTER_ENABLE,
        .FilterBank = 0,
        .FilterFIFOAssignment = CAN_FILTER_FIFO0,
        .FilterMode = CAN_FILTERMODE_IDMASK,
        .FilterIdHigh = 0,
        .FilterIdLow = 0,
        .FilterMaskIdHigh = 0,
        .FilterMaskIdLow = 0,
        .FilterScale = CAN_FILTERSCALE_32BIT,
        .SlaveStartFilterBank = 14,
    };
    return HAL_CAN_ConfigFilter(hhcan, &can_filt);
}
void BSP_CAN_Init()
{
    BSP_CAN_Filter_Init(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_Start(&hcan1);
}

HAL_StatusTypeDef BSP_CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef CanTX=
    {
        .StdId = StdId,
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .DLC = len,
    };
    return HAL_CAN_AddTxMessage(hcan,&CanTX,data,(uint32_t *)CAN_TX_MAILBOX0);
}

uint8_t  BSP_CAN_RXFlag(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxFifoFillLevel(hcan,CAN_RX_FIFO0)>0)
    {
        return 1;
    }
    else 
    return 0;
}

void BSP_CAN_Receive(CAN_HandleTypeDef *hcan, uint32_t *StdId, uint8_t *data, uint8_t *len)
{
    CAN_RxHeaderTypeDef CanRxMSG;
    HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&CanRxMSG,data);
    if(CanRxMSG.IDE == CAN_ID_STD)
    {
        *StdId = CanRxMSG.StdId;
    }
    else if(CanRxMSG.IDE == CAN_ID_EXT)
    {
        //
    }
    if(CanRxMSG.RTR == CAN_RTR_DATA)
    {
        *len = CanRxMSG.DLC;
    }
    else if(CanRxMSG.RTR == CAN_RTR_REMOTE)
    {
        //
    }
}

void BSP_GM6020_SETVOL(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1ff):(0x2ff);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (v1>>8)&0xff;
  tx_data[1] =    (v1)&0xff;
  tx_data[2] = (v2>>8)&0xff;
  tx_data[3] =    (v2)&0xff;
  tx_data[4] = (v3>>8)&0xff;
  tx_data[5] =    (v3)&0xff;
  tx_data[6] = (v4>>8)&0xff;
  tx_data[7] =    (v4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

void BSP_GM6020_SETCUR(uint8_t id_range, int16_t i1, int16_t i2, int16_t i3, int16_t i4)
{
  CAN_TxHeaderTypeDef tx_header;
  uint8_t             tx_data[8];
    
  tx_header.StdId = (id_range == 0)?(0x1fe):(0x2fe);
  tx_header.IDE   = CAN_ID_STD;
  tx_header.RTR   = CAN_RTR_DATA;
  tx_header.DLC   = 8;

  tx_data[0] = (i1>>8)&0xff;
  tx_data[1] =    (i1)&0xff;
  tx_data[2] = (i2>>8)&0xff;
  tx_data[3] =    (i2)&0xff;
  tx_data[4] = (i3>>8)&0xff;
  tx_data[5] =    (i3)&0xff;
  tx_data[6] = (i4>>8)&0xff;
  tx_data[7] =    (i4)&0xff;
  HAL_CAN_AddTxMessage(&hcan1, &tx_header, tx_data,(uint32_t*)CAN_TX_MAILBOX0); 
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    uint8_t rdata[8]={0};
  if(hcan == &hcan1)
  {
    HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, rdata);
    switch(RxHeader.StdId)
    {
      case 0x205:
      {
        MotorData.ID = RxHeader.StdId-0X204;
        MotorData.angle = rdata[0]<<8 | rdata[1];
        MotorData.rpm = -(rdata[2]<<8 | rdata[3]);//顺时针正
        MotorData.current = rdata[4]<<8 | rdata[5];
        MotorData.temp = rdata[6];
        MotorData.realANGLE = (float)MotorData.angle / ANGLERATIO * 360.0f; // 电机角度转换为实际角度
        MotorData.realSPD = (float)MotorData.rpm / 60.0f*6.283185307; // 电机转速转换为实际转速
        MotorData.realCUR = (float)MotorData.current / CURRATIO; // 电流转换为实际电流
        break;
      }
    }
  }
}
