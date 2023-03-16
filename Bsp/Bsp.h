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



//#include "KEY_bsp.h"//����
#include "Beep_bsp.h"//������
#include "Motor_bsp.h"//���̵��
//#include "Tracking_bsp.h"
#include "Usart_bsp.h"//����
#include "IRQHandler_bsp.h"//�ж�
#include "lcd.h"//LCD��Ļ
#include "Stepmotor_bsp.h"//�������
#include "Location.h"//ѭ��
#include "Control_Logic.h"//�����߼�

//�궨��
#define free 1//�����־����
#define busy 0//�����־æµ

#define Work_Debug_flag 1
//1,Working2,Debug

//��ʼ������
void Bsp_Init(void);
extern void QK_delay_ms(uint16_t time);//ֱ����ʱ

#endif

