/*
 * @Author: niutu
 * @Date: 2021-02-07 14:20:38
 * @LastEditTime: 2021-05-05 17:46:45
 * @LastEditors: NiuTu
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\Usart_bsp.c
 */




#include "Usart_bsp.h"
#include "Bsp.h"
#include "stm32f1xx_it.h"
#include <cstring>
//ȫ�ֱ���
volatile uint8_t rx_len_1,rx_len_2,rx_len_3 = 0;  //����һ֡���ݵĳ���
volatile uint8_t recv_end_flag_1,recv_end_flag_2,recv_end_flag_3 = 0; //һ֡���ݽ�����ɱ�־
uint8_t RxBuff_1[16] = {0};//����1������
uint8_t RxBuff_2[16] = {0};//����2������
uint8_t RxBuff_3[16] = {0};//����3������

uint8_t RxBuff_1_get[16] = {0};//����1������
uint8_t RxBuff_2_get[16] = {0};//����2������
uint8_t RxBuff_3_get[16] = {0};//����3������

uint16_t openmv_buf[6];//openmv����

uint16_t Rack_task_1[3];//����1����(�ϲ�)
uint16_t Rack_task_2[3];//����2����
uint16_t Rack_fact_1[3];//����1ʵ��λ��
uint16_t Rack_fact_2[3];//����2ʵ��λ��



uint16_t SG_solo_tar[3] = {770,1380,2005};//��������675��1500��2285
uint8_t SG_tx_com[7]={0x55, 0x55, 0x05, 0x06, 0x00, 0x01, 0x00};//���������
uint8_t SG_solo_tx_com[10]={0x55,0x55,0x08,0x03,0x01,0xff,0xff,0x08,0x00,0x00};//�������

uint8_t SG_rx_com;//����� ����ָ��

/**
 * @description: ��ʼ���򿪽���
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Usart_Init(void)
{
	//Ҫ�ȴ���һ�ν���
	if(HAL_UART_Receive_DMA(&huart1, RxBuff_1, 16)!= HAL_OK)
		Error_Handler();
	if(HAL_UART_Receive_DMA(&huart2, RxBuff_1, 16)!= HAL_OK)
		Error_Handler();
	if(HAL_UART_Receive_DMA(&huart3, RxBuff_1, 16)!= HAL_OK)
		Error_Handler();

	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);//ʹ��IDLE�ж�
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);//
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);//
}

/**
 * @description: �����жϻص�
 * @author: NiuTu
 * @param {UART_HandleTypeDef} *UartHandle
 * @return {*}
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle)
{
}

/**
 * @description: �����жϻص�
 * @author: NiuTu
 * @param {UART_HandleTypeDef} *UartHandle
 * @return {*}
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{/*
	if(UartHandle == &huart1)
	{
			uint32_t tmp_flag = 0;
	uint32_t temp;
	tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
	if((tmp_flag != RESET))//idle��־����λ
	{	 
		__HAL_UART_CLEAR_IDLEFLAG(&huart1);				//�����־λ
		HAL_UART_DMAStop(&huart1); 						// ֹͣDMA���䣬��ֹ
		temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);	// ��ȡDMA��δ��������ݸ���   
		rx_len_1 =  16 - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
		recv_end_flag_1 = 1;	// ������ɱ�־λ��1	
	}
		
	}
	if(UartHandle == &huart2)
	{
		uint32_t tmp_flag = 0;
		uint32_t temp;
		tmp_flag =__HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE); //��ȡIDLE��־λ
		if((tmp_flag != RESET))//idle��־����λ
		{	 
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);				//�����־λ
			HAL_UART_DMAStop(&huart2); 						// ֹͣDMA���䣬��ֹ
			temp = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);	// ��ȡDMA��δ��������ݸ���   
			rx_len_2 =  16 - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
			recv_end_flag_2 = 1;	// ������ɱ�־λ��1	
		}
	}
	if(UartHandle == &huart3)
	{
		uint32_t tmp_flag = 0;
		uint32_t temp;
		tmp_flag =__HAL_UART_GET_FLAG(&huart3,UART_FLAG_IDLE); //��ȡIDLE��־λ
		if((tmp_flag != RESET))//idle��־����λ
		{	 
			__HAL_UART_CLEAR_IDLEFLAG(&huart3);				//�����־λ
			HAL_UART_DMAStop(&huart3); 						// ֹͣDMA���䣬��ֹ
			temp = __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);	// ��ȡDMA��δ��������ݸ���   
			rx_len_3 =  16 - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
			recv_end_flag_3 = 1;	// ������ɱ�־λ��1	
		}
	}*/
}

/**
 * @description: MG65���ݽ���
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Usart_rx_MG65(void)
{
	if(recv_end_flag_1 == 1)  //������ɱ�־
	{
		if(rx_len_1 == 0x08)
		{
			Rack_task_1[0] = *(RxBuff_1_get + 0)-48;
        	Rack_task_1[1] = *(RxBuff_1_get + 1)-48;
        	Rack_task_1[2] = *(RxBuff_1_get + 2)-48;
			Rack_task_2[0] = *(RxBuff_1_get + 4)-48;
			Rack_task_2[1] = *(RxBuff_1_get + 5)-48;
			Rack_task_2[2] = *(RxBuff_1_get + 6)-48;		
        }
		recv_end_flag_1 = 0;//������ս�����־λ
		memset(RxBuff_1_get,0,rx_len_1);
		rx_len_1 = 0;//�������
	}
}
/**
 * @description: openmv���ݽ���
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Usart_rx_openmv(void)
{
	if(recv_end_flag_2 == 1)  //������ɱ�־
	{
		if(rx_len_2 == 12)
		{
			openmv_buf[0] = *(uint16_t*)(RxBuff_2_get + 0);
			openmv_buf[1] = *(uint16_t*)(RxBuff_2_get + 2);
        	//openmv_buf_fir[0] = *(uint16_t*)(RxBuff_2 + 2);
        	openmv_buf[2] = *(uint16_t*)(RxBuff_2_get + 4);
			openmv_buf[3] = *(uint16_t*)(RxBuff_2_get + 6);
			//openmv_buf_fir[1] = *(uint16_t*)(RxBuff_2 + 6);
        	openmv_buf[4] = *(uint16_t*)(RxBuff_2_get + 8);
			openmv_buf[5] = *(uint16_t*)(RxBuff_2_get + 10);
        	//openmv_buf_fir[2] = *(uint16_t*)(RxBuff_2 + 10);			
        }
		recv_end_flag_2 = 0;//������ս�����־λ
		memset(RxBuff_2_get,0,rx_len_2);
		rx_len_2 = 0;//�������
	}
}
/**
 * @description: ������ư巵��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Usart_rx_SG(void)
{
	if(recv_end_flag_3 == 1)  //������ɱ�־
	{
		if(rx_len_3 == 7)
		{
			SG_rx_com = RxBuff_3[3];//��ý��յ�ָ��
        }
		recv_end_flag_3 = 0;//������ս�����־λ
		memset(RxBuff_3,0,rx_len_3);
		rx_len_3 = 0;//�������
	}
}

/**
 * @description: ���ж�����һ��
 * @author: NiuTu
 * @param {uint8_t} ID �������id
 * @return {*}
 */
void Usart_tx_SG(uint8_t ID)
{
	*(SG_tx_com + 4) = ID;  
	HAL_UART_Transmit_DMA(&huart3, SG_tx_com, 7);
}
/**
 * @description: ָ������˶�
 * @author: NiuTu
 * @param {uint8_t} ID ���id
 * @param {uint16_t} tim ������ɺ�ʱ
 * @param {uint16_t} tar Ŀ��Ƕȣ�1500����
 * @return {*}
 */
void SG_move_tx(uint8_t ID,uint16_t tim,uint16_t tar)
{
	*(SG_solo_tx_com + 5) = tim&0xff;
	*(SG_solo_tx_com + 6) = (tim>>8)&0xff;
	*(SG_solo_tx_com + 7) = ID;
	*(SG_solo_tx_com + 8) = tar&0xff;
	*(SG_solo_tx_com + 9) = (tar>>8)&0xff;	
   // uint8_t Tx_com[10] = {0x55,0x55,0x08,0x03,0x01,tim&0xff,(tim>>8)&0xff,ID,tar&0xff,(tar>>8)&0xff};
    HAL_UART_Transmit_DMA(&huart3, SG_solo_tx_com, 10);
} 
/********************************************/



