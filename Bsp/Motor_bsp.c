/*
 * @Author: niutu
 * @Date: 2021-01-25 16:45:36
 * @LastEditTime: 2021-05-06 00:08:51
 * @LastEditors: NiuTu
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\Motor_bsp.c
 */
/*
���Ŷ���
����������
Encoder_1A			PA8
Encoder_1B			PA9
Encoder_2A			PA6
Encoder_2B			PA7
Encoder_3A			PD12
Encoder_3B			PD13
Encoder_4A			PA0
Encoder_4B			PA1
PWM����
Motor_ch1			PC6
Motor_ch2			PC7
Motor_ch3			PC8
Motor_ch4			PC9
��������
Motor_1				PG5
Motor_2				PG6
Motor_3				PG7
Motor_4				PG8
*/
#include "Motor_bsp.h"
#include "Bsp.h"


//ȫ�ֱ�������
uint8_t PID_Mode;//pidģʽѡ��

//������
EncoderTypeDef Encoder_Speed;//��������ֵ
//pwm
MortorTypedef Mortor1,Mortor2,Mortor3,Mortor4;//������Ʋ�����pwmд��ֵ��Ŀ���ٶȣ���
Incremental_PID_TypeDef 	Mortor1_IncrementalPID,	//���pid����
							Mortor2_IncrementalPID,
							Mortor3_IncrementalPID,
							Mortor4_IncrementalPID;

MoveDebugTypedef 			Speed_Debug;//������Բ���

/******************************* �ƶ����� ********************************/
/**
 * @description: �ƶ����Ƴ�ʼ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Move_Control_Init(void)
{
	Encoder_Init();//��������ʼ��
	PWM_Init();//pwm��ʼ��
	PID_Init();//pid������ʼ��
}


/**
 * @description: �ٶȿ���
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void MovePID_Control(void)
{
	Speed_Collect();
	#if (Speed_Debug_Flag == 1)//pid�ٶ�ģʽ
		//Ŀ���ٶȸ���ṹ��
		Mortor1_IncrementalPID.EX_I_Val = Mortor1.Speed_Ideal;
		Mortor2_IncrementalPID.EX_I_Val = Mortor2.Speed_Ideal;
		Mortor3_IncrementalPID.EX_I_Val = -Mortor3.Speed_Ideal;
		Mortor4_IncrementalPID.EX_I_Val = -Mortor4.Speed_Ideal;
		//��ǰ�ٶȸ���ṹ��
		Mortor1_IncrementalPID.Now_P_Val = Encoder_Speed.encoder1;
		Mortor2_IncrementalPID.Now_P_Val = Encoder_Speed.encoder2;
		Mortor3_IncrementalPID.Now_P_Val = Encoder_Speed.encoder3;
		Mortor4_IncrementalPID.Now_P_Val = Encoder_Speed.encoder4;
		Mortor1_IncrementalPID.Now_I_Val = Encoder_Speed.encoder1;
		Mortor2_IncrementalPID.Now_I_Val = Encoder_Speed.encoder2;
		Mortor3_IncrementalPID.Now_I_Val = Encoder_Speed.encoder3;
		Mortor4_IncrementalPID.Now_I_Val = Encoder_Speed.encoder4;	
		//pid����
		Simple_Incremental_PID(&Mortor1_IncrementalPID);
		Simple_Incremental_PID(&Mortor2_IncrementalPID);
		Simple_Incremental_PID(&Mortor3_IncrementalPID);
		Simple_Incremental_PID(&Mortor4_IncrementalPID);
		//�������
		Mortor1.Pwm_Write = Mortor1_IncrementalPID.Output_Val;
		Mortor2.Pwm_Write = Mortor2_IncrementalPID.Output_Val;
		Mortor3.Pwm_Write = Mortor3_IncrementalPID.Output_Val;
		Mortor4.Pwm_Write = Mortor4_IncrementalPID.Output_Val;
	#elif (Speed_Debug_Flag == 2)//����pidģʽ
		//�����ٶȸ���ṹ��
		Mortor1_IncrementalPID.EX_I_Val = Speed_Debug.Mortor1_IdealSpeed;
		Mortor2_IncrementalPID.EX_I_Val = Speed_Debug.Mortor2_IdealSpeed;
		Mortor3_IncrementalPID.EX_I_Val = -Speed_Debug.Mortor3_IdealSpeed;
		Mortor4_IncrementalPID.EX_I_Val = -Speed_Debug.Mortor4_IdealSpeed;
		//pid����
		Mortor1_IncrementalPID.Now_I_Val = Encoder_Speed.encoder1;
		Mortor2_IncrementalPID.Now_I_Val = Encoder_Speed.encoder2;
		Mortor3_IncrementalPID.Now_I_Val = Encoder_Speed.encoder3;
		Mortor4_IncrementalPID.Now_I_Val = Encoder_Speed.encoder4;
		//pid����
		Simple_Incremental_PID(&Mortor1_IncrementalPID);
		Simple_Incremental_PID(&Mortor2_IncrementalPID);
		Simple_Incremental_PID(&Mortor3_IncrementalPID);
		Simple_Incremental_PID(&Mortor4_IncrementalPID);
		//�������
		Mortor1.Pwm_Write = Mortor1_IncrementalPID.Output_Val;
		Mortor2.Pwm_Write = Mortor2_IncrementalPID.Output_Val;
		Mortor3.Pwm_Write = Mortor3_IncrementalPID.Output_Val;
		Mortor4.Pwm_Write = Mortor4_IncrementalPID.Output_Val;
	#elif  (Speed_Debug_Flag == 3)//pwm����ģʽ
		//�������
		Mortor1.Pwm_Write = Speed_Debug.Mortor1_WritePWM;
		Mortor2.Pwm_Write = Speed_Debug.Mortor2_WritePWM;
		Mortor3.Pwm_Write = Speed_Debug.Mortor3_WritePWM;
		Mortor4.Pwm_Write = Speed_Debug.Mortor4_WritePWM;
	#endif
	PWM_Write();
}


/************************************* PID *************************************/

/*
 * @name:void PID_Init(void);
 * @function: pid��ʼ��
 * @param:none
 * @return:none
 */
void PID_Init(void)
{
	//PID������ʼ��
	//λ��ʽpid����
	//A��
	Mortor1_IncrementalPID.Kp_P = 15;
	Mortor1_IncrementalPID.Ki_P = 6;
	Mortor1_IncrementalPID.Kd_P = 6;
	//B��
	Mortor2_IncrementalPID.Kp_P = 15;
	Mortor2_IncrementalPID.Ki_P = 6;
	Mortor2_IncrementalPID.Kd_P = 8;
	//C��
	Mortor3_IncrementalPID.Kp_P = 15;
	Mortor3_IncrementalPID.Ki_P = 6;
	Mortor3_IncrementalPID.Kd_P = 8;
	//D��
	Mortor4_IncrementalPID.Kp_P = 15;
	Mortor4_IncrementalPID.Ki_P = 6;
	Mortor4_IncrementalPID.Kd_P = 8;
	
	//����ʽpid����
	Mortor1_IncrementalPID.Kp_I = 18;
	Mortor1_IncrementalPID.Ki_I = 4;
	Mortor1_IncrementalPID.Kd_I = 6;
	//B��
	Mortor2_IncrementalPID.Kp_I = 18;
	Mortor2_IncrementalPID.Ki_I = 4;
	Mortor2_IncrementalPID.Kd_I = 6;
	//C��
	Mortor3_IncrementalPID.Kp_I = 18;
	Mortor3_IncrementalPID.Ki_I = 4;
	Mortor3_IncrementalPID.Kd_I = 6;
	//D��
	Mortor4_IncrementalPID.Kp_I = 18;
	Mortor4_IncrementalPID.Ki_I = 4;
	Mortor4_IncrementalPID.Kd_I = 6;
	
	//��ʼ��
	Mortor1_IncrementalPID.EX_P_Val = Mortor2_IncrementalPID.EX_P_Val
	= Mortor3_IncrementalPID.EX_P_Val = Mortor4_IncrementalPID.EX_P_Val = 0;

	Mortor1_IncrementalPID.Now_P_Val = Mortor2_IncrementalPID.Now_P_Val
	= Mortor3_IncrementalPID.Now_P_Val = Mortor4_IncrementalPID.Now_P_Val = 0;

	Mortor1_IncrementalPID.Output_Val = Mortor2_IncrementalPID.Output_Val
	= Mortor3_IncrementalPID.Output_Val = Mortor4_IncrementalPID.Output_Val = 0;

	Mortor1_IncrementalPID.Error_P_Now = Mortor2_IncrementalPID.Error_P_Now
	= Mortor3_IncrementalPID.Error_P_Now = Mortor4_IncrementalPID.Error_P_Now = 0;
	Mortor1_IncrementalPID.Error_P_Last = Mortor2_IncrementalPID.Error_P_Last
	= Mortor3_IncrementalPID.Error_P_Last = Mortor4_IncrementalPID.Error_P_Last = 0;
	
	Mortor1_IncrementalPID.Error_P_Integral = Mortor2_IncrementalPID.Error_P_Integral
	= Mortor3_IncrementalPID.Error_P_Integral = Mortor4_IncrementalPID.Error_P_Integral = 0;

	Mortor1_IncrementalPID.Dead_zone = Mortor2_IncrementalPID.Dead_zone
	= Mortor3_IncrementalPID.Dead_zone = Mortor4_IncrementalPID.Dead_zone = 2;
	
	Mortor1_IncrementalPID.Output_Max = Mortor2_IncrementalPID.Output_Max
	= Mortor3_IncrementalPID.Output_Max = Mortor4_IncrementalPID.Output_Max = 100;
	
	Mortor1_IncrementalPID.Output_Min = Mortor2_IncrementalPID.Output_Min
	= Mortor3_IncrementalPID.Output_Min = Mortor4_IncrementalPID.Output_Min = -100;

}
#define E_LIMIT 50//ƫ���޷�
/*
 * @name:void Simple_Incremental_PID(Incremental_PID_TypeDef *IncrPID);
 * @function: PID����
 * @param:none
 * @return:none
 */
void Simple_Incremental_PID(Incremental_PID_TypeDef *IncrPID)
{
	int16_t p,i,d;
/*λ��ʽ*//*
	IncrPID->Error_P_Now = IncrPID->EX_P_Val-IncrPID->Now_P_Val;//�����ȡ
	IncrPID->Error_P_Integral += IncrPID->Error_P_Now;//�ۻ�����ƫ��
		//ƫ���޷�
	if(IncrPID->Error_P_Now > E_LIMIT)
	{
		IncrPID->Error_P_Now = E_LIMIT;
	}
	if(IncrPID->Error_P_Now < -E_LIMIT)
	{
		IncrPID->Error_P_Now = -E_LIMIT;
	}*/
/*
	//��������
	if(IncrPID->Error_P_Now > -IncrPID->Dead_zone
		&&IncrPID->Error_P_Now < IncrPID->Dead_zone)
	{
		IncrPID->Error_P_Now = 0;
	}

	//����
	p = IncrPID->Kp_P*IncrPID->Error_P_Now;
	i = IncrPID->Ki_P*IncrPID->Error_P_Integral;
	d = IncrPID->Kd_P*(IncrPID->Error_P_Now - IncrPID->Error_P_Last);
	IncrPID->Output_Val = (p+i+d)/100;
	IncrPID->Error_P_Last = IncrPID->Error_P_Now;//��ǰƫ��д���ϴ�ƫ��
	
	if((IncrPID->EX_P_Val==0)&(IncrPID->Error_P_Now==0))//ɲ����Խ��Ӧ
	{
		IncrPID->Output_Val = 0;
		IncrPID->Error_P_Integral = 0;
	}
	*/
/*����ʽ*/

	IncrPID->Error_I_Now = IncrPID->EX_I_Val-IncrPID->Now_I_Val;//�����ȡ
	if(IncrPID->Error_I_Now > -IncrPID->Dead_zone
		&&IncrPID->Error_I_Now < IncrPID->Dead_zone)//��������
	{
		IncrPID->Error_I_Now = 0;
	}
	p = IncrPID->Kp_I*(IncrPID->Error_I_Now - IncrPID->Error_I_Last);
	i = IncrPID->Ki_I*IncrPID->Error_I_Now;
	d = IncrPID->Kd_I*(IncrPID->Error_I_Before + IncrPID->Error_I_Now - 2*IncrPID->Error_I_Last);

	IncrPID->Incr_Val = (int16_t)(p+i+d)/100;//�ٶ��������

	IncrPID->Output_Val += IncrPID->Incr_Val;//��������

	if((IncrPID->EX_I_Val==0)&(IncrPID->Error_I_Now==0))//ɲ����Խ��Ӧ
	{
		IncrPID->Output_Val = 0;
		IncrPID->Incr_Val=0;//������λ
	}
	
	IncrPID->Error_I_Before = IncrPID->Error_I_Last;//�ϴ�ƫ��д�����ϴ�ƫ��
	IncrPID->Error_I_Last = IncrPID->Error_I_Now;//��ǰƫ��д���ϴ�ƫ��

	//�޷�
	if(IncrPID->Output_Val >IncrPID->Output_Max)
		IncrPID->Output_Val =  IncrPID->Output_Max;
	if(IncrPID->Output_Val <IncrPID->Output_Min)
		IncrPID->Output_Val =  IncrPID->Output_Min;

}


/*
 * @name:void bsp_SpeedSum(v[3]);
 * @function:�ٶ��ںϣ�ǰ��,����,��ת
 * @parameter:ʸ����������[3]
 * @return:none
 */
void bsp_SpeedSum(int16_t *i)
{
	Mortor1.Speed_Ideal = i[0] + i[1] - i[2];
	Mortor2.Speed_Ideal = i[0] - i[1] - i[2];
	Mortor3.Speed_Ideal = i[0] - i[1] + i[2];
	Mortor4.Speed_Ideal = i[0] + i[1] + i[2];
}

/******************************** Encoder *******************************/
/*
 * @name:void Encoder_Init(void);
 * @function: ��������ʼ��
 * @param:none
 * @return:none
 */
void Encoder_Init(void)
{
	//������ͨ����
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
}

int16_t Read_Encoder(uint8_t TIMX)
{
   int Encoder_TIM;    
   switch(TIMX)
	 {
		 case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		 case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;	
		 case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
		 case 5:  Encoder_TIM= (short)TIM5 -> CNT;  TIM5 -> CNT=0;break;
		 default: Encoder_TIM=0;
	 }
	return Encoder_TIM;
}

/*
 * @name:void Speed_Collect(void);
 * @function: ��ȡ�ٶ�
 * @param:none
 * @return:none
 */
void Speed_Collect(void)
{
	//read value
	Encoder_Speed.encoder1 = Read_Encoder(2);
	Encoder_Speed.encoder2 = Read_Encoder(3);
	Encoder_Speed.encoder3 = Read_Encoder(4);
	Encoder_Speed.encoder4 = Read_Encoder(5);
	
//change encoder direction
//	Mortor1.Speed_Read =-Encoder_Speed.encoder1;
//	Mortor2.Speed_Read =Encoder_Speed.encoder2;
//	Mortor3.Speed_Read =-Encoder_Speed.encoder3;
//	Mortor4.Speed_Read =Encoder_Speed.encoder4;
}



/*********************************** PWM **********************************/
/*
 * @name:void PWM_Init(void);
 * @function: pwm��ʼ��
 * @param:none
 * @return:none
 */
void PWM_Init(void)
{
	//pwmͨ����
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_4);
}

/*
 * @name:void PWM_Write(void);
 * @function:���дpwm
 * @parameter:ռ�ձ�����pwm[]
 * @return:none
 */
void PWM_Write(void)
{
	pwm_Driver(Mortor1.Pwm_Write, Motor_ch1_GPIO_Port, Motor_ch1_Pin, 1);
	pwm_Driver(Mortor2.Pwm_Write, Motor_ch2_GPIO_Port, Motor_ch2_Pin, 2);
	pwm_Driver(Mortor3.Pwm_Write, Motor_ch3_GPIO_Port, Motor_ch3_Pin, 3);
	pwm_Driver(Mortor4.Pwm_Write, Motor_ch4_GPIO_Port, Motor_ch4_Pin, 4);
}	

/*
 * @name:void bsp_Driver(int16_t v, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
 * @function:�����������
 * @parameter:ռ�ձ�v
 * @return:none
 */
void pwm_Driver(int16_t v, GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin,uint8_t channel)
{
	if(v < 0)									
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_SET);
		switch(channel)
			{
			case 1 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 100 + v);
				//TIM2-> CCR1 =( 100 + v );
			break;
			case 2 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 100 + v);
				//TIM2-> CCR2 =( 100 + v );
			break;
			case 3 :
				//TIM2-> CCR3 =( 100 + v );
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 100 + v);
			break;
			case 4 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 100 + v);
				//TIM2-> CCR4 =( 100 + v );
			break;
			default :
			break;
			}
	}
	else if	(v > 0)
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		switch(channel)
			{
			case 1 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, v);
				//TIM2-> CCR1 = v;
			break;
			case 2 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, v);
				//TIM2-> CCR2 = v;
			break;
			case 3 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, v);
				//TIM2-> CCR3 = v;
			break;
			case 4 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, v);
				//TIM2-> CCR4 = v;
			break;
			default :
			break;
			}
	}
	else
	{
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, GPIO_PIN_RESET);
		switch(channel)
			{
			case 1 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, 0);
				//TIM2-> CCR1 = 0;
			break;
			case 2 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_2, 0);
				//TIM2-> CCR2 = 0;
			break;
			case 3 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_3, 0);
				//TIM2-> CCR3 = 0;
			break;
			case 4 :
				__HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_4, 0);
				//TIM2-> CCR4 = 0;
			break;
			default :
			break;
			}
	}
}







