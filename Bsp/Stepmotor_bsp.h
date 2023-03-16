/*
 * @Author: your name
 * @Date: 2021-01-29 21:49:30
 * @LastEditTime: 2021-03-25 00:02:22
 * @LastEditors: NiuTu
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\Logistics_robot\code\logistics_robot\Bsp\Stepmotor_bsp.h
 */
#ifndef _STEPMOTOR_BSP_H
#define _STEPMOTOR_BSP_H

#include "Bsp.h"

#define MICRO 2//细分数
#define STEP_TIM TIM2


/*结构体*/
//步进电机状态结构体
typedef struct
{

	uint8_t Flag;//完成标志
	uint16_t Toggle_Pulse;//比较输出周期
	uint16_t Pulse_Tar;//目标脉冲数
	float Pulse_Now;//当前累积脉冲
	float Step_angle;//步进角，1.8/细分数
	uint32_t channel;//电机通道
}StepmotorTypeDef;
extern StepmotorTypeDef StepMotor_A,StepMotor_B,StepMotor_C;


#define Stepmotor_Dir_GPIO      GPIOE//步进电机方向控制gpio
#define Stepmotor_Dir_Pin       GPIO_PIN_8//步进电机方向控制引脚

#define Stepmotor_forward()     HAL_GPIO_WritePin(Stepmotor_Dir_GPIO, Stepmotor_Dir_Pin,GPIO_PIN_SET)//步进电机正转
#define Stepmotor_reversal()    HAL_GPIO_WritePin(Stepmotor_Dir_GPIO, Stepmotor_Dir_Pin,GPIO_PIN_RESET)//步进电机反转

void Stepmotor_Init(void);
void Pulse_putput(uint16_t pulse_count,StepmotorTypeDef *motor);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);


#endif



