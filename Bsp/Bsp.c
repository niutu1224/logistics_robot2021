


#include "Bsp.h"




/**
 * @Function: Bsp_Init(void)
 * @Description: ��������ĳ�ʼ��
 * @Calls: Move_Control_Init()
 * @Calls: bsp_InitIRQHanlder()
 * @Called By: main()
 */
void Bsp_Init(void)
{	
//	bsp_InitKey();
//	bsp_InitLed();
//	bsp_InitBeep();
	Usart_Init();//
	LCD_Init();//LCD��Ļ��ʼ����320x240��
	Move_Control_Init();//�ƶ����Ƴ�ʼ��������pwm��pid��encoder
	Stepmotor_Init();//���������ʼ��
	Bsp_IRQHanlder_Init();//�򿪶�ʱ���жϣ��ŵ����
	Location_line_init();//��λ�ṹ���ʼ��

	run_flag = 0;//���������б�־
	
	Usart_tx_SG(0);//��е�۸�λ
	HAL_Delay(2300);
	SG_move_tx(11,100,1500);
}



void QK_delay_ms(uint16_t time)//�Ǻķ���Դ��ʱ
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;  //�Լ�����
      while(i--) ;    
   }
}




































































































