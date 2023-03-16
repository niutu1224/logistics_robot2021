/*
 * @Author: NiuTu
 * @Date: 2021-03-26 00:28:35
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\Beep_bsp.c
 */

#include "Beep_bsp.h"
#include "Bsp.h"


/**
 * @description: ÎËÃùÆ÷
 * @author: NiuTu
 * @param {uint8_t} mode
 * @return {*}
 */
void Beep(uint8_t mode)
{
	switch(mode)
	{
		case 0:
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			QK_delay_ms(20);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			QK_delay_ms(50);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			QK_delay_ms(20);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			break;
		}
		case 1:
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			QK_delay_ms(50);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			break;
		}
		case 2:
		{
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			QK_delay_ms(100);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			QK_delay_ms(50);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_SET);
			QK_delay_ms(20);
			HAL_GPIO_WritePin(BEEP_GPIO_Port,BEEP_Pin,GPIO_PIN_RESET);
			break;
		}
	
	}
}




