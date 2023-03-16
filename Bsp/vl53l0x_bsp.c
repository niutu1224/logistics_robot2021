#include "vl53l0x_bsp.h"
#include "main.h"

/*�й��ⲿ����
stm32f4xx_it.c�ﶨ�� USART1_IRQHandler �еĿ��лص�����
*/
#define TRUE 1
#define FALSE 0


uint8_t USART1_TX_FLAG = 0;//������ɱ�־λ�������0
uint8_t USART1_RC_FLAG = 0;//������ɱ�־λ�������0
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

uint8_t LR[42];//�������ݶ�
uint16_t LR_T[6];//�������ݶ�
void bsp_InitVl53l0x(void)
{
	//HAL_UART_Receive_DMA(&huart1, RxBuff, 4);
	//__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
}




/*
 * @name: Get_range
 * @function:����ģ����modbus����ͨѶ
 * @parameter:LR_Tx������buff��ַ��LR_Rx������buff��ַ
 * @return:none
 */
/*
void Get_range(uint8_t *LR_Tx,uint8_t *LR_Rx)
{
	while(USART1_RC_FLAG);//�ϴεĽ���δ��ɵȴ�
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
 * @function:�����жϻص�����
 * @parameter:huart,���ں�;
 * @return:none
 */
/*void HAL_UART_IdleCallback(UART_HandleTypeDef *huart)  
{

	__HAL_UART_CLEAR_IDLEFLAG(huart);//�����жϱ�־λ
	HAL_UART_DMAStop(huart);//�ر�dma����
	
}
*/

/***********************/
/*
 * @name: UartTxData
 * @function:���ڷ�������
 * @parameter:huart,���ں�;add�����ŵ�ַ:size�����ų���
 * @return:none
 */
void UartTxData(UART_HandleTypeDef *huart, uint8_t *add, const uint32_t size)
{
	while(USART1_TX_FLAG);//����δ��ɵȴ�

	USART1_TX_FLAG = 1;//DMA���ͱ�־æµ
	HAL_UART_Transmit_DMA(huart, add, size);//dma��������
}
/*
 * @name: HAL_UART_TxCpltCallback
 * @function:���ڷ����ж�
 * @parameter:huart,���ں�;
 * @return:none
 */

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        USART1_TX_FLAG = 0;//DMA���ͱ�־����
    }
}
/***************************/
/*
 * @name: UartRcData
 * @function:���ڽ�������
 * @parameter:huart,���ں�;
 * @return:none
 */
void UartRxData(UART_HandleTypeDef *huart, uint8_t *add, const uint32_t size)
{    
	HAL_UART_Receive_DMA(huart, add, size);
	USART1_RC_FLAG = 1;//���ձ�־æµ
}
/*
 * @name: HAL_UART_RxCpltCallback
 * @function:���ڽ����ж�
 * @parameter:huart,���ں�;
 * @return:none
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        USART1_RC_FLAG = 0;//DMA���ձ�־����
    }
}



