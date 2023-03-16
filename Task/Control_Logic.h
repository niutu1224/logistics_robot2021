/*
 * @Author: NiuTu
 * @Date: 2021-02-10 11:39:41
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Task\Control_Logic.h
 */
#ifndef _CONTROL_LOGIC_H_
#define _CONTROL_LOGIC_H_
#include "Bsp.h"

extern int8_t direction_X;
extern int8_t direction_Y;
extern int8_t direction_Z;
extern int16_t move[3];//�ƶ�����ʸ����
extern int32_t STEP_Way[6];//�ǲ�����
extern int16_t tube_adjust[3];//����������ʸ��
extern int16_t sum_move[3];//��ʸ��
extern uint8_t run_flag;//�ܶ���־λ
extern uint8_t task_number;//����˳���
extern uint8_t task_mode;//����ģʽ
extern uint8_t Debug_mode;//��������ģʽ
extern uint8_t Task_Flag;
extern uint8_t Integral_move_flag;//�ǲ��˶���־λ
extern uint8_t Patrol_line_flag;//Ѳ��������־
extern int16_t coordinate[2];//��������һ�񳤶���300
void  Coordinate_mix(void);
uint8_t Integral_move(int32_t x, int32_t y,int32_t z,int32_t V_x,int32_t V_y,int32_t V_z);
uint8_t Err_move(int16_t x, int16_t y);
void Patrol_line(uint8_t line_xy);
void Task_mode(uint8_t mode);
void Working(void);
void Task_start(void);
void Task_stop(void);
void Task_way1(void);
void Task_way2(void);
void Task_way3(void);
void Task_way4(void);
void Task_way5(void);
void Task_way6(void);
void GM65_getnum(void);
void Stop_adjust(void);
void Get_fir_body(void);
void Down_body(void);
void Down_2_body(void);
void Down_3_body(void);
void Up_body(void);
void turn_L90(void);
void turn_R90(void);
void SG_Dubug(void);
void First_get(void);//һ��۲��ȡ
void Second_get(void);//����۲��ȡ

void First_prepare(void);//һ��ץȡ׼��
void Second_prepare(void);//����ץȡ׼��

void First_body_get(void);//һ��ץȡ
void Second_body_get(void);//����ץȡ

void max_adjust(void);//��ȷ��λ
#endif // _CONTROL_LOGIC_H_


