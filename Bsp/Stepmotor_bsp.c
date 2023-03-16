/*
 * @Author: niutu
 * @Date: 2021-01-29 21:49:07
 * @LastEditTime: 2021-03-25 00:03:06
 * @LastEditors: NiuTu
 * @Description: �����������
 * @FilePath: \MDK-ARMd:\project\Logistics_robot\code\logistics_robot\Bsp\Stepmotor_bsp.c
 */



#include "Bsp.h"
#include "Stepmotor_bsp.h"
//������1.8��
#define MICRO 2//ϸ����
#define STEP_TIM TIM2
/*���״̬�ṹ��*/
StepmotorTypeDef StepMotor_A,StepMotor_B,StepMotor_C;

/**
 * @Function: Stepmotor_Init(void)
 * @Description: ��ʼ���������������tim����Ƚ��ж�ʹ�ܣ�
 * @Calls: TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState)
 * @Called By:  Bsp_Init(void)
 */
void Stepmotor_Init(void)
{
    //����ṹ�����
    StepMotor_A.Flag = 1;
    StepMotor_A.channel = TIM_CHANNEL_1;
    StepMotor_B.Flag = 1;
    StepMotor_B.channel = TIM_CHANNEL_2;
    StepMotor_C.Flag = 1;
    StepMotor_C.channel = TIM_CHANNEL_3;

    HAL_TIM_Base_Start(&htim2);//ȷ����ʱ��
	TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_1,TIM_CCx_DISABLE);//ʧ�ܱȽ����
    TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_2,TIM_CCx_DISABLE);//ʧ�ܱȽ����
    TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_3,TIM_CCx_DISABLE);//ʧ�ܱȽ����

    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//ʹ�ܱȽ�����ж�
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);//ʹ�ܱȽ�����ж�
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);//ʹ�ܱȽ�����ж�

//��������̧�𣬸�λ������
}



/**
 * @description: ����ָ����Ŀ����
 * @author: NiuTu
 * @param {uint16_t} pulse_count ����������
 * @param {TIM_TypeDef} *TIM ѡ��ʱ��/TIM2
 * @param {uint32_t} channel ѡ��ͨ��/TIM_CHANNEL_1
 * @return {*}
 */
//ִ�к������жϻص� 
/*
if(StepMotor_A.Flag)
{
    Pulse_putput(100);
}
*/
void Pulse_putput(uint16_t pulse_count,StepmotorTypeDef *motor )
{
    motor->Flag = 0;//������б�־��0
    motor->Pulse_Tar = pulse_count * 2;//˫���Ĳ�������Ŀ������*2
    TIM_CCxChannelCmd(STEP_TIM,motor->channel,TIM_CCx_ENABLE);//ʹ�ܱȽ�ͨ��
}

/**
 * @description: �Ƚ�����жϻص�����
 * @author: NiuTu
 * @param {TIM_HandleTypeDef} *htim��ʱ�����ָ��
 * @return {*}
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
    uint32_t count;
    uint32_t tmp;
    count = __HAL_TIM_GET_COUNTER(htim);//��ȡ����ֵ

    switch(htim -> Channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1:
        {
            tmp = 0xFFFF & (count+StepMotor_A.Toggle_Pulse);//�趨�������ڣ�����
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,tmp);//�趨�µıȽ���ֵ
            StepMotor_A.Pulse_Now+= 0.5;//����������
            StepMotor_A.Pulse_Tar--;
            if(StepMotor_A.Pulse_Tar <= 0)
            {
                TIM_CCxChannelCmd(STEP_TIM,
                                    StepMotor_A.channel,
                                    TIM_CCx_DISABLE);//ʧ�ܱȽ����
                StepMotor_A.Flag = 1;
            }
            break;
        }
        case HAL_TIM_ACTIVE_CHANNEL_2:
        {
            tmp = 0xFFFF & (count+StepMotor_B.Toggle_Pulse);
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,tmp);//�趨�µıȽ���ֵ
            StepMotor_B.Pulse_Now+= 0.5;//����������
            StepMotor_B.Pulse_Tar--;
            if(StepMotor_B.Pulse_Tar <= 0)
            {
                TIM_CCxChannelCmd(STEP_TIM,
                                    StepMotor_B.channel,
                                    TIM_CCx_DISABLE);//ʧ�ܱȽ����
                StepMotor_B.Flag = 1;
            }
            break;
        }
        case HAL_TIM_ACTIVE_CHANNEL_3:
        {
            tmp = 0xFFFF & (count+StepMotor_C.Toggle_Pulse);
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,tmp);//�趨�µıȽ���ֵ
            StepMotor_C.Pulse_Now+= 0.5;//����������
            StepMotor_C.Pulse_Tar--;
            if(StepMotor_C.Pulse_Tar <= 0)
            {
                TIM_CCxChannelCmd(STEP_TIM,
                                    StepMotor_C.channel,
                                    TIM_CCx_DISABLE);//ʧ�ܱȽ����
                StepMotor_C.Flag = 1;
            }
            break;
        }
    }
}
