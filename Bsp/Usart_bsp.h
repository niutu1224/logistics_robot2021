/*
 * @Author: NiuTu
 * @Date: 2021-02-07 14:21:22
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\Usart_bsp.h
 */
#ifndef _USART_BSP_H_
#define _USART_BSP_H_

#include"Bsp.h"


extern uint8_t _dbg_TXBuff[10];

extern volatile uint8_t rx_len_1,rx_len_2,rx_len_3;  //接收一帧数据的长度
extern volatile uint8_t recv_end_flag_1,recv_end_flag_2,recv_end_flag_3; //一帧数据接收完成标志
extern uint8_t RxBuff_1[16] ;//串口1缓冲区
extern uint8_t RxBuff_2[16] ;//串口2缓冲区
extern uint8_t RxBuff_3[16] ;//串口3缓冲区

extern uint8_t RxBuff_1_get[16];//串口1缓冲区
extern uint8_t RxBuff_2_get[16];//串口2缓冲区
extern uint8_t RxBuff_3_get[16];//串口3缓冲区

extern uint16_t openmv_buf[6];//openmv数据
extern uint16_t Rack_task_1[3];//货架1任务
extern uint16_t Rack_task_2[3];//货架2任务
extern uint16_t Rack_fact_1[3];//货架1实际位置
extern uint16_t Rack_fact_2[3];//货架2实际位置


extern uint8_t SG_tx_com[7];
extern uint8_t SG_solo_tx_com[10];
extern uint16_t SG_solo_tar[3];
extern uint8_t SG_rx_com;//舵机板 返回指令
void Usart_Init(void);
void Usart_rx_MG65(void);
void Usart_rx_openmv(void);                                                                                  
void Usart_rx_SG(void);
void Usart_tx_SG(uint8_t ID);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle);
void Woreless_Transmit(void);
void uart3_printf(const char *format,...);
void SG_move_tx(uint8_t ID,uint16_t tim,uint16_t tar);
#endif /* _USART_BSP_H_ */




