/*
 * @Author: NiuTu
 * @Date: 2021-01-25 16:45:36
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\IRQHandler_bsp.c
 */
#include "IRQHandler_bsp.h"

#include "bsp.h"


/**
 * @description: 定时器中断初始化
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Bsp_IRQHanlder_Init(void)
{
	
	__HAL_TIM_CLEAR_IT(&htim6, TIM_IT_UPDATE);//开中断的函数之前清除中断标志位防bug

	HAL_TIM_Base_Start_IT(&htim6);//打开定时器中断
}
/**
 * @description: 定时器中断服务函数
 * @author: NiuTu
 * @param {TIM_HandleTypeDef} *htim
 * @return {*}
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim -> Instance == TIM6)
	{
		Usart_rx_SG();
		//openmv_display();
		
		Usart_rx_openmv();//接收颜色识别
		IO_Updata();//循迹数据更新
		Location_line();//巡线处理
		Patrol_line(Patrol_line_flag);
		Coordinate_mix();//坐标融合
		
		if(Integral_move_flag == busy)//记步运动控制
		{
			if(STEP_Way[0] == 0)
			{
				move[0] = 0;
			}
			else
			{
				move[0] = STEP_Way[1];
				STEP_Way[0] -= STEP_Way[1];
			}
			if(STEP_Way[2] == 0)
			{
				move[1] = 0;
			}
			else
			{
				move[1] = STEP_Way[3];
				STEP_Way[2] -= STEP_Way[3];
			}
			if(STEP_Way[5] == 0)
			{
				move[2] = 0;
			}
			else
			{
				move[2] = STEP_Way[4];
				STEP_Way[5] -= STEP_Way[4];
			}
		}
		sum_move[0] = move[0] + tube_adjust[0];
		sum_move[1] = move[1] + tube_adjust[1];
		sum_move[2] = move[2] + tube_adjust[2];

		bsp_SpeedSum(sum_move);
		MovePID_Control();//电机pid驱动
		//Woreless_Transmit();//发送循迹数据
	}
}

/**
 * @description: 软件开关中断
 * @author: NiuTu
 * @param {uint16_t} GPIO_Pin
 * @return {*}
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == ALL_key_Pin)//引脚是上拉的
	{
		QK_delay_ms(5);
		if(HAL_GPIO_ReadPin(ALL_key_GPIO_Port,ALL_key_Pin)==GPIO_PIN_RESET)//接通
		{
			run_flag = 1;//接通开车
			Beep(Be);
		}
		if(HAL_GPIO_ReadPin(ALL_key_GPIO_Port,ALL_key_Pin)==GPIO_PIN_SET)//接通
		{
			run_flag = 0;//断开停车
			Beep(BzzBe);
		}

	}
}

