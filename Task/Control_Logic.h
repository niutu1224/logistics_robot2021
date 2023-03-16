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
extern int16_t move[3];//移动控制矢量，
extern int32_t STEP_Way[6];//记步长度
extern int16_t tube_adjust[3];//传感器矫正矢量
extern int16_t sum_move[3];//总矢量
extern uint8_t run_flag;//跑动标志位
extern uint8_t task_number;//任务顺序号
extern uint8_t task_mode;//任务模式
extern uint8_t Debug_mode;//调试任务模式
extern uint8_t Task_Flag;
extern uint8_t Integral_move_flag;//记步运动标志位
extern uint8_t Patrol_line_flag;//巡线修正标志
extern int16_t coordinate[2];//场内坐标一格长度是300
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
void First_get(void);//一层观察获取
void Second_get(void);//二层观察获取

void First_prepare(void);//一层抓取准备
void Second_prepare(void);//二层抓取准备

void First_body_get(void);//一层抓取
void Second_body_get(void);//二层抓取

void max_adjust(void);//精确定位
#endif // _CONTROL_LOGIC_H_


