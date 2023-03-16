/*
 * @Author: niutu
 * @Date: 2021-01-29 21:49:07
 * @LastEditTime: 2021-03-25 00:03:06
 * @LastEditors: NiuTu
 * @Description: 步进电机驱动
 * @FilePath: \MDK-ARMd:\project\Logistics_robot\code\logistics_robot\Bsp\Stepmotor_bsp.c
 */



#include "Bsp.h"
#include "Stepmotor_bsp.h"
//步进角1.8度
#define MICRO 2//细分数
#define STEP_TIM TIM2
/*电机状态结构体*/
StepmotorTypeDef StepMotor_A,StepMotor_B,StepMotor_C;

/**
 * @Function: Stepmotor_Init(void)
 * @Description: 初始化步进电机驱动（tim输出比较中断使能）
 * @Calls: TIM_CCxChannelCmd(TIM_TypeDef *TIMx, uint32_t Channel, uint32_t ChannelState)
 * @Called By:  Bsp_Init(void)
 */
void Stepmotor_Init(void)
{
    //电机结构体参数
    StepMotor_A.Flag = 1;
    StepMotor_A.channel = TIM_CHANNEL_1;
    StepMotor_B.Flag = 1;
    StepMotor_B.channel = TIM_CHANNEL_2;
    StepMotor_C.Flag = 1;
    StepMotor_C.channel = TIM_CHANNEL_3;

    HAL_TIM_Base_Start(&htim2);//确定定时器
	TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_1,TIM_CCx_DISABLE);//失能比较输出
    TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_2,TIM_CCx_DISABLE);//失能比较输出
    TIM_CCxChannelCmd(TIM1,TIM_CHANNEL_3,TIM_CCx_DISABLE);//失能比较输出

    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);//使能比较输出中断
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);//使能比较输出中断
    HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);//使能比较输出中断

//加入向上抬起，复位脉冲数
}



/**
 * @description: 发射指定数目脉冲
 * @author: NiuTu
 * @param {uint16_t} pulse_count 发射脉冲数
 * @param {TIM_TypeDef} *TIM 选择时钟/TIM2
 * @param {uint32_t} channel 选择通道/TIM_CHANNEL_1
 * @return {*}
 */
//执行后会进入中断回调 
/*
if(StepMotor_A.Flag)
{
    Pulse_putput(100);
}
*/
void Pulse_putput(uint16_t pulse_count,StepmotorTypeDef *motor )
{
    motor->Flag = 0;//电机运行标志置0
    motor->Pulse_Tar = pulse_count * 2;//双节拍步进，将目标脉冲*2
    TIM_CCxChannelCmd(STEP_TIM,motor->channel,TIM_CCx_ENABLE);//使能比较通道
}

/**
 * @description: 比较输出中断回调函数
 * @author: NiuTu
 * @param {TIM_HandleTypeDef} *htim定时器句柄指针
 * @return {*}
 */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef* htim)
{
    uint32_t count;
    uint32_t tmp;
    count = __HAL_TIM_GET_COUNTER(htim);//获取计数值

    switch(htim -> Channel)
    {
        case HAL_TIM_ACTIVE_CHANNEL_1:
        {
            tmp = 0xFFFF & (count+StepMotor_A.Toggle_Pulse);//设定脉冲周期，调速
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_1,tmp);//设定新的比较数值
            StepMotor_A.Pulse_Now+= 0.5;//电机脉冲计数
            StepMotor_A.Pulse_Tar--;
            if(StepMotor_A.Pulse_Tar <= 0)
            {
                TIM_CCxChannelCmd(STEP_TIM,
                                    StepMotor_A.channel,
                                    TIM_CCx_DISABLE);//失能比较输出
                StepMotor_A.Flag = 1;
            }
            break;
        }
        case HAL_TIM_ACTIVE_CHANNEL_2:
        {
            tmp = 0xFFFF & (count+StepMotor_B.Toggle_Pulse);
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_2,tmp);//设定新的比较数值
            StepMotor_B.Pulse_Now+= 0.5;//电机脉冲计数
            StepMotor_B.Pulse_Tar--;
            if(StepMotor_B.Pulse_Tar <= 0)
            {
                TIM_CCxChannelCmd(STEP_TIM,
                                    StepMotor_B.channel,
                                    TIM_CCx_DISABLE);//失能比较输出
                StepMotor_B.Flag = 1;
            }
            break;
        }
        case HAL_TIM_ACTIVE_CHANNEL_3:
        {
            tmp = 0xFFFF & (count+StepMotor_C.Toggle_Pulse);
            __HAL_TIM_SET_COMPARE(htim,TIM_CHANNEL_3,tmp);//设定新的比较数值
            StepMotor_C.Pulse_Now+= 0.5;//电机脉冲计数
            StepMotor_C.Pulse_Tar--;
            if(StepMotor_C.Pulse_Tar <= 0)
            {
                TIM_CCxChannelCmd(STEP_TIM,
                                    StepMotor_C.channel,
                                    TIM_CCx_DISABLE);//失能比较输出
                StepMotor_C.Flag = 1;
            }
            break;
        }
    }
}
