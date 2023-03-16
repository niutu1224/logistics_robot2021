/*
 * @Author: niutu
 * @Date: 2021-01-26 20:15:36
 * @LastEditTime: 2021-04-20 15:53:07
 * @LastEditors: NiuTu
 * @Description:lcd屏幕的显示任务
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\LCD\lcd_task.h
 */


#ifndef _LCD_TASK_H
#define _LCD_TASK_H

#include "Bsp.h"


extern uint8_t Number_display_flag;

void Number_display (void);
void Data_display(void);
void intToStr(uint8_t a[],int16_t m);
int16_t intlen(int16_t m);
void DrawTestPage(uint8_t *str);

char* itoa(int num, char* str,int radix);
//char itoa(int n);
#endif


