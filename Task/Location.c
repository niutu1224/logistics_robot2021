/*
 * @Author: niutu
 * @Date: 2021-02-03 18:04:12
 * @LastEditTime: 2021-04-13 20:47:45
 * @LastEditors: NiuTu
 * @Description:λ��ȷ��
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Task\Location.c
 */
#include "Location.h"
#include "Bsp.h"
//�궨��


//ȫ�ֱ���
/*
    A
D       B
    C
*///��������abcd
uint8_t Gray_level_A[4]; 
uint8_t Gray_level_B[4]; 
uint8_t Gray_level_C[4]; 
uint8_t Gray_level_D[4]; 

//�ṹ������
GurytubeTypeDef A,B,C,D;//�ĸ��������ṹ��
//����
/**
 * @description: �������ṹ���ʼ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Location_line_init(void)
{
    A.ID = 1;//��дid
    B.ID = 2;
    C.ID = 3;
    D.ID = 4;
    A.num_update_flag = 1;
    B.num_update_flag = 1;
    C.num_update_flag = 1;
    D.num_update_flag = 1;
    A.scan_off_dir = -1;
    B.scan_off_dir = -1;
    C.scan_off_dir =  1;
    D.scan_off_dir =  1;
}

/**
 * @description: ���»Ҷȵ�״̬������
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void IO_Updata(void)
{
    Gray_level_A[0] = HAL_GPIO_ReadPin(A_1_GPIO_Port, A_1_Pin);
    Gray_level_A[1] = HAL_GPIO_ReadPin(A_2_GPIO_Port, A_2_Pin);
    Gray_level_A[2] = HAL_GPIO_ReadPin(A_3_GPIO_Port, A_3_Pin);
    Gray_level_A[3] = HAL_GPIO_ReadPin(A_4_GPIO_Port, A_4_Pin);
    Gray_level_B[0] = HAL_GPIO_ReadPin(B_1_GPIO_Port, B_1_Pin);
    Gray_level_B[1] = HAL_GPIO_ReadPin(B_2_GPIO_Port, B_2_Pin);
    Gray_level_B[2] = HAL_GPIO_ReadPin(B_3_GPIO_Port, B_3_Pin);
    Gray_level_B[3] = HAL_GPIO_ReadPin(B_4_GPIO_Port, B_4_Pin);
    Gray_level_C[0] = HAL_GPIO_ReadPin(C_1_GPIO_Port, C_1_Pin);
    Gray_level_C[1] = HAL_GPIO_ReadPin(C_2_GPIO_Port, C_2_Pin);
    Gray_level_C[2] = HAL_GPIO_ReadPin(C_3_GPIO_Port, C_3_Pin);
    Gray_level_C[3] = HAL_GPIO_ReadPin(C_4_GPIO_Port, C_4_Pin);
    Gray_level_D[0] = HAL_GPIO_ReadPin(D_1_GPIO_Port, D_1_Pin);
    Gray_level_D[1] = HAL_GPIO_ReadPin(D_2_GPIO_Port, D_2_Pin);
    Gray_level_D[2] = HAL_GPIO_ReadPin(D_3_GPIO_Port, D_3_Pin);
    Gray_level_D[3] = HAL_GPIO_ReadPin(D_4_GPIO_Port, D_4_Pin);
}

/**
 * @description: Ѱ�ߺ�������
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Location_line(void)
{
    GuryTube_state(Gray_level_A,&A);
    GuryTube_state(Gray_level_B,&B);
    GuryTube_state(Gray_level_C,&C);
    GuryTube_state(Gray_level_D,&D);//״̬��д
    /*
    Pressure_Line(&A);
    Pressure_Line(&B);
    Pressure_Line(&C);
    Pressure_Line(&D);//ѹ�߼���
    */
    ScanLine(&A);
    ScanLine(&B);
    ScanLine(&C);
    ScanLine(&D);//ɨ���ж�
}


/**
 * @description: ״̬��д
 * @author: NiuTu
 * @param {uint8_t} *Gray_level
 * @param {GurytubeTypeDef} *id
 * @return {*}
 */
uint8_t GuryTube_state(uint8_t *Gray_level,GurytubeTypeDef *id )
{
    //�˶�����״̬����Ϊ��������Է���
    switch (id->ID)//ѡ�񴫸���
    {
    case 1:
        id->dive_dir_X = -direction_X;//
        id->dive_dir_Y =  direction_Y;//
        break;
    case 2:
        id->dive_dir_X = -direction_Y;//
        id->dive_dir_Y = -direction_X;//
        break;
    case 3:
        id->dive_dir_X =  direction_X;//
        id->dive_dir_Y = -direction_Y;//
        break;
    case 4:
        id->dive_dir_X =  direction_Y;//
        id->dive_dir_Y =  direction_X;//
        break;
    }
    //������״̬����
    if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 0)//����������XXXX
    {
        id->state = 0;//״̬���0
        id->Pressure_Line_state = 0;//ѹ��״̬����
    }
    else if(*(Gray_level + 0) == 1 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 0)//������ɨ��OXXX
    {
        id->state = 1;//״̬���1
        id->Pressure_Line_state = 0;//ѹ��״̬����
        id->Last_on_state = 1;//���ִ���״̬����
    }
    else if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 1 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 0)//������ɨ��XOXX
    {
        id->state = 2;//״̬���2
        id->Pressure_Line_state = 0;//ѹ��״̬����
        id->Last_on_state = 2;//���ִ���״̬����
    }
    else if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 1 && *(Gray_level + 3) == 0)//������ɨ��XXOX
    {
        id->state = 3;//״̬���3
        id->Pressure_Line_state = 0;//ѹ��״̬����
        id->Last_on_state = 3;//���ִ���״̬����
    }
    else if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 1)//������ɨ��XXXO
    {
        id->state = 4;//״̬���4
        id->Pressure_Line_state = 0;//ѹ��״̬����
        id->Last_on_state = 4;//���ִ���״̬����
    }
    else if(*(Gray_level + 0) == 1 && *(Gray_level + 3) == 1)//���˸ߵ磬������ѹ��,OOOO
    {
        id->state = 5;//״̬���5
        id->Pressure_Line_state = 1;//ѹ��״̬����
    }
    else//����ο�
    {
        id->state = 6;//״̬���6
    }
    //�������ݵĸ���
    if(sum_move[0] < 0)
    {
        direction_Y = -1;
    }
    else if(sum_move[0] > 0)
    {
        direction_Y = 1;
    }
    else
    direction_Y = 0;
        //�������ݵĸ���
    if(sum_move[1] < 0)
    {
        direction_X = -1;
    }
    else if(sum_move[1] > 0)
    {
        direction_X = 1;
    }
    else
    direction_X = 0;
    return 0;
}
/**
 * @description: ѹ�߼���
 * @author: NiuTu
 * @param {GurytubeTypeDef} *id ������id��ѡB,D
 * @return {*}
 */
void Pressure_Line(GurytubeTypeDef *id)
{
    if(id->state == 6)//�������ж�
    {
        return;
    }
    if(!id->Last_Pressure_Line_state == id->Pressure_Line_state)//ѹ��״̬����
    {
        if(id->Pressure_Line_state == 1)  //����Ϊ����
        {
            id->scan_num_Y = id->scan_num_Y + id->dive_dir_Y;//ѹ�߼���
            id->Last_dir_Y = id->dive_dir_Y;//��¼���߷���
            
        }
        else                //����Ϊ����
        {
            if(id->Last_dir_Y != id->dive_dir_Y)//���߷��������߷���ͬ�����¼���
            {
                id->scan_num_Y = id->scan_num_Y + id->dive_dir_Y;//ѹ�߼���
            }
        }
        id->Last_Pressure_Line_state = id->Pressure_Line_state;//����ѹ��״̬
    }   
}

/**
 * @description: ɨ���ж�
 * @author: NiuTu
 * @param {GurytubeTypeDef} *id ������id
 * @return {*}
 */
void ScanLine(GurytubeTypeDef *id)
{
    /*ɸȥ���ж����*/
    if(id->state == 6 )
    {
        return;//�������ж�
    }
    if(id->state == 5)
    {
		//id->err = 0;//��ִ��У׼����
        return;//ѹ��û���жϵı�Ҫ
    }
    
    if(id->Last_state != id->state) //ɨ��״̬����
    {
        if(id->state != 0)              //����Ϊ����
        {
            if(id->scan_state == 0)     //���ߵ�ɨ��
            {
                if(id->state == 1)//������
                {
                    id->scan_state = 1;//���Ϊɨ������
                    id->scan_on_dir = -1;//��ʼ��ɨ�߷���
                    id->err = -4;
                    if(id->scan_off_dir == id->scan_on_dir)//���߷��������߷���ͬ��
                    {
                        id->scan_num_X--;//�󴥷���ȥɨ����
                    }
                }
                else if(id->state == 4)//������
                {
                    id->scan_state = 1;//���Ϊɨ������
                    id->scan_on_dir =  1;//��ʼ��ɨ�߷��������ߵ�ʱ���ж��Ƿ�����
                    id->err =  4;
                    if(id->scan_off_dir == id->scan_on_dir)//���߷��������߷���ͬ��
                    {
                        id->scan_num_X++;//�Ҵ�������ɨ����
                    }     
                }
                else//�м�������ѹ�ߵ�ǰ״̬������
                return;
            }
            else                        //ɨ������µ�����
            {
                /*Ѳ��״̬��д*/
                switch(id->state)
                {
                case 1://���޾Ȼ�
                    id->err = -4;
                    break;
                case 2://ƫ�򴫸�������
                    id->err = -1;
                    break;
                case 3://ƫ�򴫸�������
                    id->err =  1;
                    break;            
                case 4://���޾Ȼ�
                    id->err =  4;
                    break;
                }
            }
        }
        else                            //����Ϊ����
        {
            //2,3�Թ����м�����
            if((id->Last_state == 2 && id->dive_dir_X == 1)||(id->Last_state == 3 && id->dive_dir_X == -1))
            {
                id->err = 0;//Ѳ�߲�ƫ
            }
			if(id->Last_state == 1 && id->dive_dir_X == 1 )//������
            {
                id->scan_state = 0;//���Ϊ����
                id->scan_off_dir = 1;//�������߷���
//                    if(id->scan_off_dir != id->scan_on_dir)//���߷��������߷�����
//                    {
//                        id->scan_num_X++;//�󴥷���ȥɨ����
//                    }
            }
            else if(id->Last_state == 4 && id->dive_dir_X ==-1)//������
            {
                id->scan_state = 0;//���Ϊɨ������
                id->scan_off_dir = -1;//�������߷��������ߵ�ʱ���ж��Ƿ�����
//                    if(id->scan_off_dir != id->scan_on_dir)//���߷��������߷�����
//                    {
//                        id->scan_num_X--;//�Ҵ�������ɨ����
//                    }     
            }
//            else if(((id->Last_state == 4 && id->dive_dir_X ==-1)
//                ||(id->Last_state == 1 && id->dive_dir_X == 1 ))
//                && id->num_update_flag)//�����뿪��������û����Ѳ��
//            {
//                id->scan_state = 0;//����
//                id->scan_off_dir = id->dive_dir_X;//�������߷���
//            }
            //�޶����ߣ�Ĭ��Ϊ�ⶪ���ھ������ٵõ��µľ������
            
            else if(id->dive_dir_X == 0)
            {
                switch(id->Last_state)
                {
                case 1://���޾Ȼ�
                    id->err = -4;
                    break;
                case 2://ƫ�򴫸�������
                    id->err = -1;
                    break;
                case 3://ƫ�򴫸�������
                    id->err =  1;
                    break;            
                case 4://���޾Ȼ�
                    id->err =  4;
                    break;
                }
            }
        }
    id->Last_state = id->state;//�����ϴ�״̬
    } 
}
/*
        if(id->err !=0)
        {
			if(id->num_update_flag)//û����Ѳ��
			{
				if(id->scan_on_dir != id->err) //ƫ�����
				{
					
				}
				id->scan_on_dir = id->err;//�����ϴ�ƫ��
			}
        }
        */



