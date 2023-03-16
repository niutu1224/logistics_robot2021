/*
 * @Author: NiuTu
 * @Date: 2021-03-21 00:34:18
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\Logistics_robot\code\logistics_robot\Task\Control_Arm.h
 */
#ifndef __CONTROL_ARM_H
#define __CONTROL_ARM_H
#include  "Control_Arm.h"
#include "Bsp.h"
//�ռ�ṹ��
typedef struct
{
	int16_t X;//��ǰx
	int16_t Y;//����y
	int32_t sqr_L;//ԭ����Ŀ�곤��ƽ��
	float L;//ԭ����Ŀ�곤��
	float theta;//ԭ����Ŀ��������x��н�
	float alpha;
	float alpha_tar;
	float beta;
	float beta_tar;
}  	Coordinate_Typedef;

extern Coordinate_Typedef now,tar;


void CordTF(Coordinate_Typedef* cord);
void AngleCalc(Coordinate_Typedef* cord);

#endif







