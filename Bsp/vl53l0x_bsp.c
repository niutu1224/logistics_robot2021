#include "vl53l0x_bsp.h"
#include "main.h"

/*有关外部定义
stm32f4xx_it.c里定义 USART1_IRQHandler 中的空闲回调函数
*/
#define TRUE 1
#define FALSE 0


uint8_t USART1_TX_FLAG = 0;//发送完成标志位，完成置0
uint8_t USART1_RC_FLAG = 0;//接收完成标志位，完成置0
uint8_t LR_T1[8] = {0x50,0x03,0x00,0x34,0x00,0x01,0xc8,0x45};
uint8_t LR_T2[8] = {0x51,0x03,0x00,0x34,0x00,0x01,0xc9,0x94};
uint8_t LR_T3[8] = {0x52,0x03,0x00,0x34,0x00,0x01,0xc9,0xa7};
uint8_t LR_T4[8] = {0x53,0x03,0x00,0x34,0x00,0x01,0xc8,0x76};
uint8_t LR_T5[8] = {0x54,0x03,0x00,0x34,0x00,0x01,0xc9,0xc1};
uint8_t LR_T6[8] = {0x55,0x03,0x00,0x34,0x00,0x01,0xc8,0x10};


uint8_t LR_R1[7];
uint8_t LR_R2[7];
uint8_t LR_R3[7];
uint8_t LR_R4[7];
uint8_t LR_R5[7];
uint8_t LR_R6[7];

uint8_t LR[42];//接收数据段
uint16_t LR_T[6];//结算数据段
void bsp_InitVl53l0x(void)
{
	//HAL_UART_Receive_DMA(&huart1, RxBuff, 4);
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}




/*
 * @name: Get_range
 * @function:与测距模块做modbus串口通讯
 * @parameter:LR_Tx，发送buff地址；LR_Rx，接收buff地址
 * @return:none
 */
/*
void Get_range(uint8_t *LR_Tx,uint8_t *LR_Rx)
{
	while(USART1_RC_FLAG);//上次的接收未完成等待
	UartTxData(&huart1, LR_Tx, 8);
	UartRxData(&huart1, LR_Rx, 7);
}
*/
void Get_range(void)
{
	uint8_t i = 0;
	uint8_t a = 3;
	

	UartTxData(&huart1, LR_T1, 8);
	HAL_Delay(1);
	UartTxData(&huart1, LR_T2, 8);
	HAL_Delay(1);
	UartTxData(&huart1, LR_T3, 8);
	HAL_Delay(1);
	UartTxData(&huart1, LR_T4, 8);
	HAL_Delay(1);
	UartTxData(&huart1, LR_T5, 8);
	HAL_Delay(1);
	UartTxData(&huart1, LR_T6, 8);
	HAL_Delay(1);
	while(USART1_RC_FLAG);
	for(i=0; i<6; i++)
	{
		*(LR_T + i) = 256 * *(LR + a) + *(LR + a + 1);
		a = a + 7;
	}	
}
/***********************/


/*
 * @name: HAL_UART_IdleCallback
 * @function:空闲中断回调函数
 * @parameter:huart,串口号;
 * @return:none
 */
/*void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)  
{

	__HAL_UART_CLEAR_IDLEFLAG(huart);//清理中断标志位
	HAL_UART_DMAStop(huart);//关闭dma传输
	
}
*/

/***********************/
/*
 * @name: UartTxData
 * @function:串口发送数据
 * @parameter:huart,串口号;add，收信地址:size，收信长度
 * @return:none
 */
void UartTxData(UART_HandleTypeDef *huart, uint8_t *add, const uint32_t size)
{
	while(USART1_TX_FLAG);//发送未完成等待

	USART1_TX_FLAG = 1;//DMA发送标志忙碌
	HAL_UART_Transmit_DMA(huart, add, size);//dma发送数据
}
/*
 * @name: HAL_UART_TxCpltCallback
 * @function:串口发送中断
 * @parameter:huart,串口号;
 * @return:none
 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        USART1_TX_FLAG = 0;//DMA发送标志空闲
    }
}
/***************************/
/*
 * @name: UartRcData
 * @function:串口接收数据
 * @parameter:huart,串口号;
 * @return:none
 */
void UartRxData(UART_HandleTypeDef *huart, uint8_t *add, const uint32_t size)
{    
	HAL_UART_Receive_DMA(huart, add, size);
	USART1_RC_FLAG = 1;//接收标志忙碌
}
/*
 * @name: HAL_UART_RxCpltCallback
 * @function:串口接收中断
 * @parameter:huart,串口号;
 * @return:none
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        USART1_RC_FLAG = 0;//DMA接收标志空闲
    }
}



