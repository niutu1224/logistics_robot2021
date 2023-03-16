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

#define MICRO 2//ϸ����
#define STEP_TIM TIM2


/*�ṹ��*/
//�������״̬�ṹ��
typedef struct
{

	uint8_t Flag;//��ɱ�־
	uint16_t Toggle_Pulse;//�Ƚ��������
	uint16_t Pulse_Tar;//Ŀ��������
	float Pulse_Now;//��ǰ�ۻ�����
	float Step_angle;//�����ǣ�1.8/ϸ����
	uint32_t channel;//���ͨ��
}StepmotorTypeDef;
extern StepmotorTypeDef StepMotor_A,StepMotor_B,StepMotor_C;


#define Stepmotor_Dir_GPIO      GPIOE//��������������gpio
#define Stepmotor_Dir_Pin       GPIO_PIN_8//������������������

#define Stepmotor_forward()     HAL_GPIO_WritePin(Stepmotor_Dir_GPIO, Stepmotor_Dir_Pin,GPIO_PIN_SET)//���������ת
#define Stepmotor_reversal()    HAL_GPIO_WritePin(Stepmotor_Dir_GPIO, Stepmotor_Dir_Pin,GPIO_PIN_RESET)//���������ת

void Stepmotor_Init(void);
void Pulse_putput(uint16_t pulse_count,StepmotorTypeDef *motor);
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim);


#endif



