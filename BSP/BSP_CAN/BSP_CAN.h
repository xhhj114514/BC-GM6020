#ifndef BSP_CAN_H
#define BSP_CAN_H

#pragma once
#include "main.h"
#include "math.h"
#include "can.h"

#define ANGLERATIO 8192.0f
#define CURRATIO 16384.0f
#define __PI     3.141592f




typedef struct 
{
    uint32_t ID;
    uint16_t angle;   // 电机角度�??0-8191�??
    int16_t rpm;        // 电机转�?�（rpm�??
    int16_t current;    // 实时电流（A�??
    uint8_t temp;    // 温度
    float a;
    float realSPD;
    float last_realSPD;
    float Filted_RealSPD;
    float realCUR;
    float Force;
    float rotation;
    float ACCANG;
}MotorData_t;

extern MotorData_t MotorData;

HAL_StatusTypeDef BSP_CAN_Filter_Init(CAN_HandleTypeDef *hhcan);
void BSP_CAN_Init();
uint8_t  BSP_CAN_RXFlag(CAN_HandleTypeDef *hcan);
HAL_StatusTypeDef BSP_CAN_Transmit(CAN_HandleTypeDef *hcan, uint32_t StdId, uint8_t *data, uint8_t len);
void BSP_CAN_Receive(CAN_HandleTypeDef *hcan, uint32_t *StdId, uint8_t *data, uint8_t *len);

void BSP_GM6020_SETVOL(uint8_t id_range, int16_t v1, int16_t v2, int16_t v3, int16_t v4);
void BSP_GM6020_SETCUR(uint8_t id_range, int16_t i1, int16_t i2, int16_t i3, int16_t i4);

#endif // !BSP_CAN_H
