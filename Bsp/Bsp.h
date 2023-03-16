/*
 * @Author: your name
 * @Date: 2021-01-25 16:45:36
 * @LastEditTime: 2021-05-04 13:22:43
 * @LastEditors: NiuTu
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Bsp\Bsp.h
 */
#ifndef __BSP_H
#define __BSP_H

#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "sys.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"



//#include "KEY_bsp.h"//按键
#include "Beep_bsp.h"//嗡鸣器
#include "Motor_bsp.h"//底盘电机
//#include "Tracking_bsp.h"
#include "Usart_bsp.h"//串口
#include "IRQHandler_bsp.h"//中断
#include "lcd.h"//LCD屏幕
#include "Stepmotor_bsp.h"//步进电机
#include "Location.h"//循迹
#include "Control_Logic.h"//控制逻辑

//宏定义
#define free 1//任务标志空闲
#define busy 0//任务标志忙碌

#define Work_Debug_flag 1
//1,Working2,Debug

//初始化函数
void Bsp_Init(void);
extern void QK_delay_ms(uint16_t time);//直接延时

#endif

