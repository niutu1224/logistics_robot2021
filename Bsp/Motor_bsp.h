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
//���Ժ궨��
#define Speed_Debug_Flag 1// 1: not debug 2:debug pid(speed) 3:debug pwm



/*********************����**********************/
//����
void Move_Control_Init(void);
void MovePID_Control(void);

/*********************������**********************/
//�ṹ��
//������1,2,3,4
typedef struct
{
	int16_t encoder1;
	int16_t encoder2;
	int16_t encoder3;
	int16_t encoder4;
}	EncoderTypeDef;
//����
extern EncoderTypeDef Encoder_Speed;//�ⲿ���� �������ٶ�
//����
void Encoder_Init(void);	//��������ʼ��
void Speed_Collect(void);	//��������

/*********************PWM**********************/
//�ṹ��
//����������ݣ�ת�٣����ţ�Ŀ���٣�
typedef struct
{
	int16_t Speed_Read;//����ת��
	int16_t Pwm_Write; //���pwm
	float Speed_Ideal;//Ŀ���ٶ�
}	MortorTypedef;
//����������
typedef struct
{
	int16_t Mortor1_IdealSpeed;//pid����
	int16_t Mortor2_IdealSpeed;
	int16_t Mortor3_IdealSpeed;
	int16_t Mortor4_IdealSpeed;
	int16_t Mortor1_WritePWM;//pwm����
	int16_t Mortor2_WritePWM;
	int16_t Mortor3_WritePWM;
	int16_t Mortor4_WritePWM;
} 	MoveDebugTypedef;

//����
extern MortorTypedef Mortor1,Mortor2,Mortor3,Mortor4;//���pid
extern MoveDebugTypedef Speed_Debug;//�����ò���

//����
void PWM_Init(void);//pwm��ʼ��
void PWM_Write(void);//дpwm
//�������
void pwm_Driver(int16_t v, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint8_t channel);

/*********************PID**********************/
extern uint8_t PID_Mode;//pidģʽѡ��
//�ṹ��
typedef struct
{
	int16_t Now_P_Val;//��ǰλ��ʽ�ٶ�
	int16_t EX_P_Val;//Ŀ��λ��ʽ�ٶ�
	int16_t Now_I_Val;//��ǰ����ʽ�ٶ�
	int16_t EX_I_Val;//Ŀ������ʽ�ٶ�
	int16_t Incr_Val;//�����ٶ�
	int16_t Output_Val;//����ٶ�

	int16_t Output_Max;//��Ҫ��ʼ��
	int16_t Output_Min;

	int16_t Error_P_Now;//��ǰƫ�λ�ã�
	int16_t Error_P_Integral;//�ۻ�ƫ�λ�ã�
	int16_t Error_P_Last;//����ƫ�λ�ã�
	int16_t Error_I_Now;//��ǰƫ�������
	int16_t Error_I_Last;//����ƫ�������
	int16_t Error_I_Before; //������ƫ�������
	int16_t Dead_zone;//����
	int16_t Kp_P;
	int16_t Ki_P;
	int16_t Kd_P;
	float Kp_I;
	float Ki_I;
	float Kd_I;
	
}	Incremental_PID_TypeDef;

//define
#define abs(x) ((x)<0? (-x):(x))//ȡ��

//����
extern Incremental_PID_TypeDef Mortor1_IncrementalPID,Mortor2_IncrementalPID,Mortor3_IncrementalPID,Mortor4_IncrementalPID;

//����
void PID_Init(void);//pid������ʼ��
void Simple_Incremental_PID(Incremental_PID_TypeDef *IncrPID);//����ʽPID����
void bsp_SpeedSum( int16_t *i); //�ٶ��ں�


#endif /* CODE_SPEED_CONTROL_H_ */



