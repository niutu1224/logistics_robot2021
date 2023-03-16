


#include "Bsp.h"




/**
 * @Function: Bsp_Init(void)
 * @Description: 所有外设的初始化
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
	LCD_Init();//LCD屏幕初始化（320x240）
	Move_Control_Init();//移动控制初始化，包含pwm，pid，encoder
	Stepmotor_Init();//步进电机初始化
	Bsp_IRQHanlder_Init();//打开定时器中断，放到最后
	Location_line_init();//定位结构体初始化

	run_flag = 0;//不开启运行标志
	
	Usart_tx_SG(0);//机械臂复位
	HAL_Delay(2300);
	SG_move_tx(11,100,1500);
}



void QK_delay_ms(uint16_t time)//非耗费资源延时
{    
   uint16_t i=0;  
   while(time--)
   {
      i=12000;  //自己定义
      while(i--) ;    
   }
}




































































































