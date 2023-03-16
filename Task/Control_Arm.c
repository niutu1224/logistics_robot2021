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
#define L1 10 //�۳�1
#define L2 10 //�۳�2

uint8_t vector_move_flag = 0;

//�ṹ��
Coordinate_Typedef  now;

/*�˶�*/


/**
 * @description: �����Ŀ������Ƕ�alpha_tar��beta_tar
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void vector_move(Coordinate_Typedef* cord)
{
    if(vector_move_flag == 0)//�˶����ƿ���
    {
        CordTF(cord);
        AngleCalc(cord);
        (now.alpha_tar - now.alpha )/(1.8/MICRO);
    }
}

/**
 * @description: �ѿ�������ת������
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
 * @description:���������Ƕ� 
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

/***********�����е�۶�����ʹ��*************************/


//����λ1
