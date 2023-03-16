/*
 * @Author:niutu
 * @Date: 2021-01-26 20:15:12
 * @LastEditTime: 2021-05-07 01:29:44
 * @LastEditors: NiuTu
 * @Description: lcd��Ļ����ʾ����
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\LCD\lcd_task.c
 */

#include "Bsp.h"
#include "lcd_task.h"
#include "gui.h"

uint8_t Number_display_flag = 0;

/**
 * @description: ɨ��������ʾ
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
			case 1:Show_Str(w,1,BLUE,WHITE,"һ",56,1);break;
			case 2:Show_Str(w,1,BLUE,WHITE,"��",56,1);break;
			case 3:Show_Str(w,1,BLUE,WHITE,"��",56,1);break;
		}
		switch(Rack_task_2[i])
		{
			case 1:Show_Str(w,100,BLUE,WHITE,"һ",56,1);break;
			case 2:Show_Str(w,100,BLUE,WHITE,"��",56,1);break;
			case 3:Show_Str(w,100,BLUE,WHITE,"��",56,1);break;
		}
		w+= 56;
	}
 	Number_display_flag = 1;
}
/**
 * @description: ������ʾ
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
	//����x
	char n[10] = {0};
	itoa((int)-A.scan_num_X, n, 10);
	Show_Str(170,1,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//����y
	itoa((int)D.scan_num_X, n, 10);
	Show_Str(190,1,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//����˳����
	itoa((int)task_number, n, 10);
	Show_Str(170,17,BLUE,WHITE,(uint8_t*)n,16,0);
	memset(n, 0, 10);
	//����ģʽ���
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


//value: Ҫת����������string: ת������ַ���,radix: ת������������2,8,10,16 ���Ƶȡ�
char* itoa(int num, char* str,int radix)
{
    char index[]="0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ";//������
    unsigned unum;//���Ҫת���������ľ���ֵ,ת�������������Ǹ���
    int i=0,j,k;//i����ָʾ�����ַ�����Ӧλ��ת��֮��i��ʵ�����ַ����ĳ��ȣ�
	//ת����˳��������ģ��������������k����ָʾ����˳��Ŀ�ʼλ��;
	//j����ָʾ����˳��ʱ�Ľ�����
 
    //��ȡҪת���������ľ���ֵ
    if(radix==10&&num<0)//Ҫת����ʮ�����������Ǹ���
    {
        unum=(unsigned)-num;//��num�ľ���ֵ����unum
        str[i++]='-';//���ַ�����ǰ������Ϊ'-'�ţ�����������1
    }
    else unum=(unsigned)num;//����numΪ����ֱ�Ӹ�ֵ��unum
 
    //ת�����֣�ע��ת�����������
    do
    {
        str[i++]=index[unum%(unsigned)radix];//ȡunum�����һλ��������Ϊstr��Ӧλ��ָʾ������1
        unum/=radix;//unumȥ�����һλ
 
    }while(unum);//ֱ��unumΪ0�˳�ѭ��
 
    str[i]='\0';//���ַ���������'\0'�ַ���c�����ַ�����'\0'������
 
    //��˳���������
    if(str[0]=='-') k=1;//����Ǹ��������Ų��õ������ӷ��ź��濪ʼ����
    else k=0;//���Ǹ�����ȫ����Ҫ����
 
    char temp;//��ʱ��������������ֵʱ�õ�
    for(j=k;j<=(i-1)/2;j++)//ͷβһһ�Գƽ�����i��ʵ�����ַ����ĳ��ȣ��������ֵ�ȳ�����1
    {
        temp=str[j];//ͷ����ֵ����ʱ����
        str[j]=str[i-1+k-j];//β����ֵ��ͷ��
        str[i-1+k-j]=temp;//����ʱ������ֵ(��ʵ����֮ǰ��ͷ��ֵ)����β��
    }
 
    return str;//����ת������ַ���
 
}

void DrawTestPage(uint8_t *str)
{
//���ƹ̶���up
LCD_Clear(WHITE);
LCD_Fill(0,0,lcddev.width,20,BLUE);
//���ƹ̶���down
LCD_Fill(0,lcddev.height-20,lcddev.width,lcddev.height,BLUE);
POINT_COLOR=WHITE;
Gui_StrCenter(0,2,WHITE,BLUE,str,16,1);//������ʾ
Gui_StrCenter(0,lcddev.height-18,WHITE,BLUE,"http://www.lcdwiki.com",16,1);//������ʾ
//���Ʋ�������
//LCD_Fill(0,20,lcddev.width,lcddev.height-20,WHITE);
}





