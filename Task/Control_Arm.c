/*
 * @Author: NiuTu
 * @Date: 2021-03-21 00:33:52
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Task\Control_Arm.c
 */

#include "Bsp.h"
#include "Control_Arm.h"
#include "math.h"

#define PI 3.14
#define L1 10 //臂长1
#define L2 10 //臂长2

uint8_t vector_move_flag = 0;

//结构体
Coordinate_Typedef  now;

/*运动*/


/**
 * @description: 输出坐目标坐标角度alpha_tar，beta_tar
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void vector_move(Coordinate_Typedef* cord)
{
    if(vector_move_flag == 0)//运动控制空闲
    {
        CordTF(cord);
        AngleCalc(cord);
        (now.alpha_tar - now.alpha )/(1.8/MICRO);
    }
}

/**
 * @description: 笛卡尔坐标转极坐标
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void CordTF(Coordinate_Typedef* cord)
{
    cord->sqr_L = cord->X * cord->X + cord->Y * cord->Y;
    cord->L = sqrt((float)cord->sqr_L);
    cord->theta = (float)atan2((double)cord->X, (double)cord->Y);
}

/**
 * @description:极坐标解算角度 
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void AngleCalc(Coordinate_Typedef* cord)
{
    float temp1,temp2;
    temp1 = (cord->sqr_L + L1*L1 - L2*L2)/(2*L1*cord->sqr_L);
    cord->alpha_tar = cord->theta + acos(temp1);
    temp2 = ((float)cord->Y - L1*sin(cord->alpha_tar))/L2;
    cord->beta_tar = asin(temp2) - cord->alpha_tar + PI/2.0f;
}

/***********舵机机械臂动作组使用*************************/


//货架位1
