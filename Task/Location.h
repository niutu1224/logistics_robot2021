/*
 * @Author: niutu
 * @Date: 2021-02-03 18:06:09
 * @LastEditTime: 2021-02-20 11:39:22
 * @LastEditors: NiuTu
 * @Description: In User Settings Edit
 * @FilePath: \MDK-ARMd:\project\Logistics_robot\code\logistics_robot\Task\Location.h
 */


#ifndef _LOCATION_H
#define _LOCATION_H
#include"Bsp.h"

//全局变量
/*
    A
D       B
    C
*/
extern uint8_t Gray_level_A[4]; //四组数组abcd
extern uint8_t Gray_level_B[4]; //四组数组abcd
extern uint8_t Gray_level_C[4]; //四组数组abcd
extern uint8_t Gray_level_D[4]; //四组数组abcd 


typedef struct
{
    uint8_t ID; //传感器id
    uint8_t state;//状态
    uint8_t Last_state;//上次状态
    uint8_t Last_on_state;//上次触发状态
	
    uint8_t Pressure_Line_state;//线状态，压线1
    uint8_t Last_Pressure_Line_state;//上轮线状态，压线1
	
    uint8_t scan_state;//扫线时黑线是否在传感器内，在线1
    int8_t  err;//扫线的偏差方向-左+右
    int8_t  scan_on_dir; //扫线上线方向方向-左+右
    int8_t  scan_off_dir;//扫线下线方向方向-左+右
	uint8_t num_update_flag;//扫线计数更新标志，循迹时候不要更新
    //（↑→）方向是传感器自身相对的要与车方向变换
    int8_t  dive_dir_X;//跳变方向x
    int8_t  dive_dir_Y;//跳变方向y
    int8_t  Last_dir_X;
    int8_t  Last_dir_Y;
    int8_t  scan_num_X;//扫线计数X
    int8_t  scan_num_Y;//扫线计数Y
}GurytubeTypeDef;
extern GurytubeTypeDef A,B,C,D;

//声明
void Location_line_init(void);//传感器结构体初始化
void Location_line(void);//寻线函数整合
void IO_Updata(void);//循迹数据刷新
uint8_t GuryTube_state(uint8_t *Gray_level,GurytubeTypeDef *id );
void Pressure_Line(GurytubeTypeDef *id);
void ScanLine(GurytubeTypeDef *id);

#endif /* _LOCATION_H */


