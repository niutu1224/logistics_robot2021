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

//ȫ�ֱ���
/*
    A
D       B
    C
*/
extern uint8_t Gray_level_A[4]; //��������abcd
extern uint8_t Gray_level_B[4]; //��������abcd
extern uint8_t Gray_level_C[4]; //��������abcd
extern uint8_t Gray_level_D[4]; //��������abcd 


typedef struct
{
    uint8_t ID; //������id
    uint8_t state;//״̬
    uint8_t Last_state;//�ϴ�״̬
    uint8_t Last_on_state;//�ϴδ���״̬
	
    uint8_t Pressure_Line_state;//��״̬��ѹ��1
    uint8_t Last_Pressure_Line_state;//������״̬��ѹ��1
	
    uint8_t scan_state;//ɨ��ʱ�����Ƿ��ڴ������ڣ�����1
    int8_t  err;//ɨ�ߵ�ƫ���-��+��
    int8_t  scan_on_dir; //ɨ�����߷�����-��+��
    int8_t  scan_off_dir;//ɨ�����߷�����-��+��
	uint8_t num_update_flag;//ɨ�߼������±�־��ѭ��ʱ��Ҫ����
    //�������������Ǵ�����������Ե�Ҫ�복����任
    int8_t  dive_dir_X;//���䷽��x
    int8_t  dive_dir_Y;//���䷽��y
    int8_t  Last_dir_X;
    int8_t  Last_dir_Y;
    int8_t  scan_num_X;//ɨ�߼���X
    int8_t  scan_num_Y;//ɨ�߼���Y
}GurytubeTypeDef;
extern GurytubeTypeDef A,B,C,D;

//����
void Location_line_init(void);//�������ṹ���ʼ��
void Location_line(void);//Ѱ�ߺ�������
void IO_Updata(void);//ѭ������ˢ��
uint8_t GuryTube_state(uint8_t *Gray_level,GurytubeTypeDef *id );
void Pressure_Line(GurytubeTypeDef *id);
void ScanLine(GurytubeTypeDef *id);

#endif /* _LOCATION_H */


