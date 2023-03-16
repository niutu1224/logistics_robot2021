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
//空间结构体
typedef struct
{
	int16_t X;//向前x
	int16_t Y;//向上y
	int32_t sqr_L;//原点与目标长度平方
	float L;//原点与目标长度
	float theta;//原点与目标连线与x轴夹角
	float alpha;
	float alpha_tar;
	float beta;
	float beta_tar;
}  	Coordinate_Typedef;

extern Coordinate_Typedef now,tar;


void CordTF(Coordinate_Typedef* cord);
void AngleCalc(Coordinate_Typedef* cord);

#endif







