/*
 * @Author: niutu
 * @Date: 2021-01-25 16:45:36
 * @LastEditTime: 2021-03-30 14:11:07
 * @LastEditors: NiuTu
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\Motor_bsp.h
 */






#ifndef __MOTOR_H
#define __MOTOR_H
#include "Bsp.h"
//调试宏定义
#define Speed_Debug_Flag 1// 1: not debug 2:debug pid(speed) 3:debug pwm



/*********************控制**********************/
//声明
void Move_Control_Init(void);
void MovePID_Control(void);

/*********************编码器**********************/
//结构体
//编码器1,2,3,4
typedef struct
{
	int16_t encoder1;
	int16_t encoder2;
	int16_t encoder3;
	int16_t encoder4;
}	EncoderTypeDef;
//参数
extern EncoderTypeDef Encoder_Speed;//外部调用 编码器速度
//声明
void Encoder_Init(void);	//编码器初始化
void Speed_Collect(void);	//更新数据

/*********************PWM**********************/
//结构体
//电机控制数据（转速，油门，目标速）
typedef struct
{
	int16_t Speed_Read;//测量转速
	int16_t Pwm_Write; //输出pwm
	float Speed_Ideal;//目标速度
}	MortorTypedef;
//调试用数据
typedef struct
{
	int16_t Mortor1_IdealSpeed;//pid调试
	int16_t Mortor2_IdealSpeed;
	int16_t Mortor3_IdealSpeed;
	int16_t Mortor4_IdealSpeed;
	int16_t Mortor1_WritePWM;//pwm调试
	int16_t Mortor2_WritePWM;
	int16_t Mortor3_WritePWM;
	int16_t Mortor4_WritePWM;
} 	MoveDebugTypedef;

//参数
extern MortorTypedef Mortor1,Mortor2,Mortor3,Mortor4;//电机pid
extern MoveDebugTypedef Speed_Debug;//调试用参数

//声明
void PWM_Init(void);//pwm初始化
void PWM_Write(void);//写pwm
//电机驱动
void pwm_Driver(int16_t v, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint8_t channel);

/*********************PID**********************/
extern uint8_t PID_Mode;//pid模式选择
//结构体
typedef struct
{
	int16_t Now_P_Val;//当前位置式速度
	int16_t EX_P_Val;//目标位置式速度
	int16_t Now_I_Val;//当前增量式速度
	int16_t EX_I_Val;//目标增量式速度
	int16_t Incr_Val;//增量速度
	int16_t Output_Val;//输出速度

	int16_t Output_Max;//需要初始化
	int16_t Output_Min;

	int16_t Error_P_Now;//当前偏差（位置）
	int16_t Error_P_Integral;//累积偏差（位置）
	int16_t Error_P_Last;//上轮偏差（位置）
	int16_t Error_I_Now;//当前偏差（增量）
	int16_t Error_I_Last;//上轮偏差（增量）
	int16_t Error_I_Before; //上上轮偏差（增量）
	int16_t Dead_zone;//死区
	int16_t Kp_P;
	int16_t Ki_P;
	int16_t Kd_P;
	float Kp_I;
	float Ki_I;
	float Kd_I;
	
}	Incremental_PID_TypeDef;

//define
#define abs(x) ((x)<0? (-x):(x))//取正

//参数
extern Incremental_PID_TypeDef Mortor1_IncrementalPID,Mortor2_IncrementalPID,Mortor3_IncrementalPID,Mortor4_IncrementalPID;

//声明
void PID_Init(void);//pid参数初始化
void Simple_Incremental_PID(Incremental_PID_TypeDef *IncrPID);//增量式PID函数
void bsp_SpeedSum( int16_t *i); //速度融合


#endif /* CODE_SPEED_CONTROL_H_ */



