/*
 * @Author:niutu
 * @Date: 2021-01-26 20:15:12
 * @LastEditTime: 2021-05-07 01:29:44
 * @LastEditors: NiuTu
 * @Description: lcd屏幕的显示任务
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\LCD\lcd_task.c
 */

#include "Bsp.h"
#include "lcd_task.h"
#include "gui.h"

uint8_t Number_display_flag = 0;

/**
 * @description: 扫码数据显示
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Number_display(void)
{
	uint8_t w = 1;
	for(uint8_t i = 0;i<3;i++)
	{
		switch(Rack_task_1[i])
		{
			case 1:Show_Str(w,1,BLUE,WHITE,"一",56,1);break;
			case 2:Show_Str(w,1,BLUE,WHITE,"二",56,1);break;
			case 3:Show_Str(w,1,BLUE,WHITE,"三",56,1);break;
		}
		switch(Rack_task_2[i])
		{
			case 1:Show_Str(w,100,BLUE,WHITE,"一",56,1);break;
			case 2:Show_Str(w,100,BLUE,WHITE,"二",56,1);break;
			case 3:Show_Str(w,100,BLUE,WHITE,"三",56,1);break;
		}
		w+= 56;
	}
 	Number_display_flag = 1;
}
/**
 * @description: 数据显示
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Data_display(void)

/*
	intToStr((uint8_t*)n,*(int16_t*)(openmv_buf + 1));
	Show_Str(170,1,BLUE,WHITE,(uint8_t*)n,12,0);
	intToStr((uint8_t*)n,*(int16_t*)(openmv_buf + 3));
	Show_Str(170,13,BLUE,WHITE,(uint8_t*)n,12,0);
	intToStr((uint8_t*)n,*(int16_t*)(openmv_buf + 5));
	Show_Str(170,25,BLUE,WHITE,(uint8_t*)n,12,0);

	intToStr((uint8_t*)n,*(int16_t*)(openmv_buf + 2));
	Show_Str(190,1,BLUE,WHITE,(uint8_t*)n,12,0);
	intToStr((uint8_t*)n,*(int16_t*)(openmv_buf + 4));
	Show_Str(190,13,BLUE,WHITE,(uint8_t*)n,12,0);
	intToStr((uint8_t*)n,*(int16_t*)(openmv_buf + 6));
	Show_Str(190,25,BLUE,WHITE,(uint8_t*)n,12,0);
*/
{
	//坐标x
	char n[10] = {0};
	itoa((int)-A.scan_num_X, n, 10);
	Show_Str(170,1,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//坐标y
	itoa((int)D.scan_num_X, n, 10);
	Show_Str(190,1,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//任务顺序编号
	itoa((int)task_number, n, 10);
	Show_Str(170,17,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//任务模式编号
	itoa((int)task_mode, n, 10);
	Show_Str(190,17,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//openmv_buff
	itoa((int)openmv_buf[1], n, 10);
	Show_Str(170,34,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	itoa((int)openmv_buf[3], n, 10);
	Show_Str(200,34,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	itoa((int)openmv_buf[5], n, 10);
	Show_Str(230,34,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);

	itoa((int)Rack_fact_1[0], n, 10);
	Show_Str(170,51,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	itoa((int)Rack_fact_1[1], n, 10);
	Show_Str(200,51,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	itoa((int)Rack_fact_1[2], n, 10);
	Show_Str(230,51,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);

	itoa((int)Rack_fact_2[0], n, 10);
	Show_Str(170,68,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	itoa((int)Rack_fact_2[1], n, 10);
	Show_Str(200,68,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	itoa((int)Rack_fact_2[2], n, 10);
	Show_Str(230,68,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
}


//value: 要转换的整数，string: 转换后的字符串,radix: 转换进制数，如2,8,10,16 进制等。
char* itoa(int num, char* str,int radix)
{
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//索引表
    unsigned unum;//存放要转换的整数的绝对值,转换的整数可能是负数
    int i=0,j,k;//i用来指示设置字符串相应位，转换之后i其实就是字符串的长度；
	//转换后顺序是逆序的，有正负的情况，k用来指示调整顺序的开始位置;
	//j用来指示调整顺序时的交换。
 
    //获取要转换的整数的绝对值
    if(radix==10&&num<0)//要转换成十进制数并且是负数
    {
        unum=(unsigned)-num;//将num的绝对值赋给unum
        str[i++]='-';//在字符串最前面设置为'-'号，并且索引加1
    }
    else unum=(unsigned)num;//若是num为正，直接赋值给unum
 
    //转换部分，注意转换后是逆序的
    do
    {
        str[i++]=index[unum%(unsigned)radix];//取unum的最后一位，并设置为str对应位，指示索引加1
        unum/=radix;//unum去掉最后一位
 
    }while(unum);//直至unum为0退出循环
 
    str[i]='\0';//在字符串最后添加'\0'字符，c语言字符串以'\0'结束。
 
    //将顺序调整过来
    if(str[0]=='-') k=1;//如果是负数，符号不用调整，从符号后面开始调整
    else k=0;//不是负数，全部都要调整
 
    char temp;//临时变量，交换两个值时用到
    for(j=k;j<=(i-1)/2;j++)//头尾一一对称交换，i其实就是字符串的长度，索引最大值比长度少1
    {
        temp=str[j];//头部赋值给临时变量
        str[j]=str[i-1+k-j];//尾部赋值给头部
        str[i-1+k-j]=temp;//将临时变量的值(其实就是之前的头部值)赋给尾部
    }
 
    return str;//返回转换后的字符串
 
}

void DrawTestPage(uint8_t *str)
{
//绘制固定栏up
LCD_Clear(WHITE);
LCD_Fill(0,0,lcddev.width,20,BLUE);
//绘制固定栏down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,2,WHITE,BLUE,str,16,1);//居中显示
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"http://www.lcdwiki.com",16,1);//居中显示
//绘制测试区域
//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
}





