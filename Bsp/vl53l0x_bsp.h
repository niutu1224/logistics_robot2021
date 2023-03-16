/*
 * @Author: your name
 * @Date: 2021-01-25 16:45:36
 * @LastEditTime: 2021-02-02 20:24:21
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\Logistics_robot\code\logistics_robot\Bsp\vl53l0x_bsp.h
 */
#ifndef __VL53L0X_BSP_H
#define __VL53L0X_BSP_H
#include "Bsp.h"

//  #define HSE_VALUE    ((uint32_t)8000000U) /*!< Value of the External oscillator in Hz */
extern uint8_t LR_T1[8];
extern uint8_t LR_T2[8];
extern uint8_t LR_T3[8];
extern uint8_t LR_T4[8];
extern uint8_t LR_T5[8];
extern uint8_t LR_T6[8];

extern uint8_t LR_R1[7];
extern uint8_t LR_R2[7];
extern uint8_t LR_R3[7];
extern uint8_t LR_R4[7];
extern uint8_t LR_R5[7];
extern uint8_t LR_R6[7];

extern uint8_t LR[42];
extern uint16_t LR_T[6];
	
void bsp_InitVl53l0x(void);
void Get_range(void);
void UartTxData(UART_HandleTypeDef *huart, uint8_t *add, const uint32_t size);
void UartRxData(UART_HandleTypeDef *huart, uint8_t *add, const uint32_t size);

#endif





