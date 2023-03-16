/*
 * @Author: NiuTu
 * @Date: 2021-02-10 11:39:27
 * @LastEditors: NiuTu
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Task\Control_Logic.c
 */


#include "Control_Logic.h"
#include "Bsp.h"



//ȫ�ֱ���
int8_t direction_X = 1;//�˶�x���򣬡�����Ϊ����ԭ�����ð��˶���λ�ŵ�����
int8_t direction_Y = 1;//�˶�y���򣬡�
int8_t direction_Z = 1;//�˶�z����˳ʱ��1

int16_t move[3] = {0,0,0};//{y��x��z}�ƶ�����ʸ����ǰ��,����,˳��

int16_t tube_adjust[3];//����������ʸ��
int16_t sum_move[3];//��ʸ��

uint8_t run_flag = 0;//�ܶ���־λ

int16_t coordinate[2];//��������һ�񳤶���300
int32_t STEP_Way[6];//�ǲ�����

 
uint8_t task_number = 0;//����˳��š�
uint8_t task_mode;//����ģʽ
uint8_t Debug_mode;//��������ģʽ

//��־λ
uint8_t Task_Flag = free;//�������
uint8_t Integral_move_flag = free;//�ǲ��˶���־λ
uint8_t Patrol_line_flag = 3;//Ѳ��������־
#define ERROR_V_MAX 2000//ƫ���˶��ٶ�
#define ERROR_V 1000//ƫ���˶��ٶ�
/**************************************��λ*****************************************/

/**
 * @description: �����ں�
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void  Coordinate_mix(void)
{
	if(C.scan_state && A.scan_state)
	{
		coordinate[0] = -A.scan_num_X * 300;
	}
	/*
	else
	{
		coordinate[0] = D.scan_num_Y * 300 + 150 +  (C.scan_num_X - D.scan_num_Y)*150
					+ (-B.scan_num_Y - D.scan_num_Y)*75;
	}*/
	if(D.scan_state && B.scan_state)
	{
		coordinate[1] = D.scan_num_X * 300;
	}
	/*
	else
	{
		coordinate[1] = A.scan_num_Y * 300 + 150 +  (D.scan_num_X - A.scan_num_Y)*150
					+ (-C.scan_num_Y - A.scan_num_Y)*75;
	}
	*/
}
/**************************************·����Ϊ*****************************************/
/**
 * @description:����ƫ���˶�
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
uint8_t Err_move(int16_t x, int16_t y)
{
	int16_t E_X = x - coordinate[0];
	int16_t E_Y = y - coordinate[1];
	static uint8_t path = 0;
	switch (path)
	{
		case 0:
		{
			if(E_X>300)
			{
				move[1] = ERROR_V_MAX;
				Patrol_line_flag = 0;
			}
			else if(E_X == 300)
			{
				move[1] = ERROR_V;
				Patrol_line_flag = 0;
			}
			else if(E_X< -300)
			{
				move[1] =-ERROR_V_MAX;
				Patrol_line_flag = 0;
			}
			else if(E_X == -300)
			{
				move[1] =-ERROR_V;
				Patrol_line_flag = 0;
			}
			else
			{
				Task_stop();
				path = 1; 
			}
			break;
		}
		case 1:
		{
			if(E_Y>300)
			{
				move[0] = ERROR_V_MAX;
				Patrol_line_flag = 1;
			}
			else if(E_Y == 300)
			{
				move[0] = ERROR_V;
				Patrol_line_flag = 1;
			}
			else if(E_Y< -300)
			{
				move[0] =-ERROR_V_MAX;
				Patrol_line_flag = 1;
			}
			else if(E_Y == -300)
			{
				move[0] =-ERROR_V;
				Patrol_line_flag = 1;
			}
			else
			{
				move[0] = 0;
				path = 0;
				return 1;
			}
			break;
		}
	}
	return 0;
}	


/**
 * @description: �ǲ��˶�
 * @author: NiuTu
 * @param {int16_t} x
 * @param {int16_t} y
 * @return {*}
 */
uint8_t Integral_move(int32_t x, int32_t y,int32_t z,int32_t V_x,int32_t V_y,int32_t V_z)
{
	static uint8_t path = 0;
	if(path == 0)
	{
		for(uint8_t i = 0;i<6;i++)
		{
			STEP_Way[i] = 0;
		}
		STEP_Way[0] = x;
		STEP_Way[1] = V_x;
		STEP_Way[2] = y;
		STEP_Way[3] = V_y;
		STEP_Way[4] = z;
		STEP_Way[5] = V_z;		
		path = 1;
		Integral_move_flag = busy;
		HAL_Delay(10);
		return 0;
	}
	else
	{
		if(move[0] == 0 && move[1] == 0 && move[2] == 0)
		{
			path = 0;
			Integral_move_flag = free;
			return 1;
		}
		else return 0;
	}
}


/**
 * @description: �˶�����һ����ѹ��
 * @author: NiuTu				  ��0
 * @param {int8_t} to_dir����	3��  ��1
 * @return {*}					  ��2
 */
uint8_t Move_to_line(int8_t to_dir)
{
	switch(to_dir)
	{
		case 0:
		{
			Patrol_line_flag = 1;//����Ѳ��
			move[0] = 400;
			move[1] = 0;
			move[2] = 0;
			if(C.state == 5)
			{
				Task_stop();
				return 1;//����
			}
			break;
		}
		case 1:
		{
			Patrol_line_flag = 0;//����Ѳ��
			move[0] = 0;
			move[1] = -400;
			move[2] = 0;
			if(D.state == 5)
			{
				Task_stop();
				return 1;//����
			}
			break;
		}
		case 2:
		{
			Patrol_line_flag = 1;//����Ѳ��
			move[0] = -400;
			move[1] = 0;
			move[2] = 0;
			if(A.state == 5)
			{
				Task_stop();
				return 1;//����
			}
			break;
		}
		case 3:
		{
			Patrol_line_flag = 0;//����Ѳ��
			move[0] = 0;
			move[1] = 400;
			move[2] = 0;
			if(B.state == 5)
			{
				Task_stop();
				return 1;//����
			}
			break;
		}
	}
	return 0;
}
/**
 * @description: Ѳ������//���ж���ִ��
 * @author: NiuTu
 * @param {uint8_t} line_xy����ѡ��0x1y
 * @return {*}
 */
void Patrol_line(uint8_t line_xy)
{
	int16_t adjust_num_x;
	int16_t adjust_num_y;
	adjust_num_x = abs(move[1]);
	adjust_num_y = abs(move[0]);
	adjust_num_x = adjust_num_x>600?adjust_num_x:600;
	adjust_num_y = adjust_num_y>600?adjust_num_y:600;
    switch (line_xy)
    {
		case 0://ѡ��x����Ѳ��
		{
			B.num_update_flag = 0;
			D.num_update_flag = 0;
			C.num_update_flag = 1;
			A.num_update_flag = 1;
			*(tube_adjust + 0) =(-B.err + D.err) * (adjust_num_x/20+10);
			*(tube_adjust + 1) = 0;//B.err*40 + D.err*40;
			*(tube_adjust + 2) =( B.err + D.err) * (adjust_num_x/20+10)*2;
			break;
		}
		case 1://ѡ��Y����Ѳ��
		{
			C.num_update_flag = 0;
			A.num_update_flag = 0;
			B.num_update_flag = 1;
			D.num_update_flag = 1;
			*(tube_adjust + 1) = (-A.err + C.err) * (adjust_num_y/20+10);
			*(tube_adjust + 0) =  0;//A.err*20 + C.err*20;
			*(tube_adjust + 2) = ( A.err + C.err) * (adjust_num_y/20+10);
			break;
		}
		case 2://���������
		{
			B.num_update_flag = 0;
			D.num_update_flag = 0;
			C.num_update_flag = 0;
			A.num_update_flag = 0;
			*(tube_adjust + 0) =-B.err*20 + D.err*20;
			*(tube_adjust + 1) =-A.err*20 + C.err*20;
			*(tube_adjust + 2) = B.err*40 + D.err*40 + A.err*40 + C.err*40;
			break;
		}
		case 3://��Ѳ��
		{
			B.num_update_flag = 0;
			D.num_update_flag = 0;
			C.num_update_flag = 0;
			A.num_update_flag = 0;
			*(tube_adjust + 0) =0;
			*(tube_adjust + 1) =0;
			*(tube_adjust + 2) =0;
			break;
		}
	}
}
	/*�޷�ʹ��ʱ��ʹ��ע���ڴ���

		case 0://ѡ��x����Ѳ��
		{
			B.num_update_flag = 0;
			D.num_update_flag = 0;
			C.num_update_flag = 1;
			A.num_update_flag = 1;
			*(tube_adjust + 0) =-B.err*20 + D.err*20;
			*(tube_adjust + 1) = 0;//B.err*40 + D.err*40;
			*(tube_adjust + 2) = B.err*40 + D.err*40;
			break;
		}
		case 1://ѡ��Y����Ѳ��
		{
			C.num_update_flag = 0;
			A.num_update_flag = 0;
			B.num_update_flag = 1;
			D.num_update_flag = 1;
			*(tube_adjust + 1) = -A.err*20 + C.err*20;
			*(tube_adjust + 0) =  0;//A.err*20 + C.err*20;
			*(tube_adjust + 2) =  A.err*40 + C.err*40;
			break;
		}
		case 2://���������
		{
			B.num_update_flag = 0;
			D.num_update_flag = 0;
			C.num_update_flag = 0;
			A.num_update_flag = 0;
			*(tube_adjust + 0) =-B.err*10 + D.err*10;
			*(tube_adjust + 1) =-A.err*10 + C.err*10;
			*(tube_adjust + 2) = B.err*20 + D.err*20 + A.err*20 + C.err*20;
			break;
		}
	*/


/**************************************�˶�����*****************************************/


/**
 * @description: ����˳��ִ�У��ŵ�ѭ����
 * @author: NiuTu
 * @param {*}
 * @return {*} 
 */
uint8_t Task[]={2,//��ʼ����
				1,
				3,//ȥɨ��
				9,//���ɨ������
				4,//�˶��������
				
				18,//�ϲ�۲�
				19,//�²�۲�
				20,//�ϼ�ȡԤ��
				1,
				16,//��ת90
				22,//�ϲ�ץȡ
				1,
				15,//��ת90
				//1,
				//10,//�ϲ�ȡ��
				5,//�˶����ּӹ���
				11,//�Ż�
				12,//�ջ�
				1,
				7,//�˶������Ʒ��
				1,
				15,//��ת90
				24,
				13,//�Ż�����
				1,
				16,//��ת90
				1,
				6,//�˶������������
				1,
				16,//��ת90
				23,//ץ��
				15,//��ת90
				5,//�˶����ּӹ���
				11,//�Ż�
				12,//�ջ�
				1,
				7,//�˶������Ʒ��
				1,
				15,//��ת90
				24,
				14,//�Ż�����
				1,
				16,//��ת90
				1,
				8,//�˶���������
				0};//����

void Working(void)
{
	if(Task_Flag)//æµ����
	{
		Beep(Be);
		task_mode = *(Task+task_number);
		Task_Flag = busy;
		task_number++;
//		if(task_number > 28)
//			run_flag = 0;
	}
}

/**
 * @description:����ģʽѡ�񣬷ŵ�ѭ����
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_mode(uint8_t mode)
{
	switch(mode)
	{
		case 0:Task_stop();break;
		case 1:Stop_adjust();break;
		case 2:Task_start();break;//��ʼ����
		case 3:Task_way1();break;//ȥɨ��
		case 4:Task_way2();break;//�˶��������
		case 5:Task_way3();break;//�˶����ּӹ���
		case 6:Task_way4();break;//�˶��������2��
		case 7:Task_way5();break;//�˶������Ʒ��
		case 8:Task_way6();break;//�˶���������
		
		case 9: GM65_getnum();break;//���ɨ������
		case 10: Get_fir_body();break;//�߲������ȡ
		case 11: Down_body();break;
		case 12: Up_body();break;
		case 13: Down_2_body();break;
		case 14: Down_3_body();break;
		
		case 15: turn_L90();break;
		case 16: turn_R90();break;
		case 17: SG_Dubug();break;

		case 18: First_get();break;//һ��۲��ȡ
		case 19: Second_get();break;//����۲��ȡ

		case 20: First_prepare();break;//һ��ץȡ׼�� 

		case 22: First_body_get();break;//һ��ץȡ
		case 23: Second_body_get();break;//����ץȡ

		case 24: max_adjust();break;//��ȷ����


	}
}
/**************************************����ģʽ*****************************************/
//0
/**
 * @description: ͣ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_stop(void)
{
	Patrol_line_flag = 3;
	move[0] = 0;
	move[1] = 0;
	move[2] = 0;
}
//1
/**
 * @description:�ڽ�������λ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Stop_adjust(void)
{
	uint8_t sum = 0;
	move[0] = 0;
	move[1] = 0;
	move[2] = 0;
	Patrol_line_flag = 2;  
	if(A.scan_state && B.scan_state && C.scan_state && D.scan_state)
	{
		A.err == 0?sum++:' ';
		A.err == 1?sum++:' ';
		A.err ==-1?sum++:' ';
		B.err == 0?sum++:' ';
		B.err == 1?sum++:' ';
		B.err ==-1?sum++:' ';
		C.err == 0?sum++:' ';
		C.err == 1?sum++:' ';
		C.err ==-1?sum++:' ';
		D.err == 0?sum++:' ';
		D.err == 1?sum++:' ';
		D.err ==-1?sum++:' ';
		if(sum >= 3)
		{
			Task_stop();
			Task_Flag = free;
		}
	}
}
//2
/**
 * @description: ��ʼ����
 * @author: NiuTu
 * @param {*}
 * @return {*}0
 */
void Task_start(void)
{
	move[0] = 400;
	move[1] = 400;
	if(A.scan_state && B.scan_state 
		&& C.scan_state && D.scan_state)
	{
		A.scan_num_X = -1;
		Task_Flag = free;
	}
}
//3
/**
 * @description: ȥɨ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_way1(void)
{
	static uint8_t path = 0;
	if(Rack_task_1[0] != 0 && Rack_task_2[0] != 0)
	{
		//Usart_tx_SG(0);//��е�۸�λ
		path = 0;
		Task_Flag = free;
	}
	
	switch(path)
	{
		case 0:
		{
			if(Err_move(2*300,2*300))
				path = 1;
			break;
		}
		case 1:
		{
			move[0] = 600;
			move[1] = 0;
			move[2] = 0;
			//Patrol_line_flag = 1;
			if(A.state == 5)
			{
				Task_stop();
				path = 0;
				Task_Flag = free;//����
			}
			break;
		}
	}
}

//4
/**
 * @description:�˶��������
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_way2(void)
{
	static uint8_t path = 0;
	switch(path)
	{
		case 0:
		{
			if(Err_move(2*300,6*300))
				path = 1;
			break;
		}
		case 1:
		{
			if(Move_to_line(3))
			{
				path = 0;
				Task_Flag = free;//����
			}
			break;
		}
	}
}
//5
/**
 * @description:�˶����ּӹ���
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_way3(void)
{	
	static uint8_t path = 0;
	switch(path)
	{
		case 0:
		{
			if(Err_move(4*300,7*300))
			{
				Task_stop();
				path = 1;
			}
			break;	
		}
		case 1:
		{
			if(Move_to_line(2))
			{
				path = 0;
				Task_Flag = free;//����
			}
			break;
		}
	}
}
//6
/**
 * @description:�˶��������2��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_way4(void)
{
	if(Err_move(2*300,6*300))
		Task_Flag = free;
}
//7
/**
 * @description:�˶������Ʒ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_way5(void)
{
	if(Err_move(6*300,4*300))
		Task_Flag = free;
}
//8
/**
 * @description: �˶���������
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Task_way6(void)
{
	static uint8_t path = 0;
	switch(path)
	{
		case 0:
		{
			A.scan_num_X = -6;
			D.scan_num_X = 4;
			path = 1;
			break;
		}
		case 1:
		{
			Usart_tx_SG(0);//��е�۸�λ
			if(Err_move(6*300,300))
			{
				Task_stop();
				path = 2;//����
			}
			break;
		}
		case 2:
		{
			SG_move_tx(11,100,1500);//���ܸ�λ
			move[0] = 0;
			move[1] = ERROR_V;
			move[2] = 0;
			if(coordinate[0] == 7*300 && B.state == 5)
			{
				path = 3;
			}
			break;
		}
		case 3:
		{
			move[0] = -ERROR_V;
			move[1] = 0;
			move[2] = 0;
			if(A.state == 5)
			{
				path = 0;
				Task_Flag = free;//����
			}
			break;
		}					
	}
}



//9
/**
 * @description: ͣ��ɨ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void GM65_getnum(void)
{
	Task_stop();
	if(Rack_task_1[0] != 0 && Rack_task_2[0] != 0)
	{
		//Usart_tx_SG(0);//��е�۸�λ
		Task_Flag = free;
	}
}

//10
/**
 * @description: ȡ�ϲ�����
 * @author: NiuTu
 * @param {*} 
 * @return {*}
 */
void Get_fir_body(void)
{
	static uint8_t path = 0;
	static uint8_t Lock_on_state = 0;//״̬��0Ѱ1��
	static uint8_t Go = 0;//����
	static uint8_t TX_SG = 1;
	static int16_t lock_v = 500;//�����ٶ�
	int8_t Err;
	
	if(path == 3)
	{
		if(Err_move(300,7*300))
		{
			Task_Flag = free;
			path = 0;
		}
		return;
	}
	else
	{
	//��д����״̬

		//����״̬��д
		
		//ɨ����Ѳ��
		if(coordinate[1] <= 4*300)
		{
			lock_v = 500;
			Lock_on_state = 0;
		}
		else if(coordinate[1] >= 7*300)
		{
			lock_v = -500;
			Lock_on_state = 0;
		}
/*
		switch(Lock_on_state)
		{
			case 0:*/
			if(Lock_on_state == 0)
			{
				Patrol_line_flag = 1;//����Ѳ��
				Lock_on_state = *(openmv_buf + (*(Rack_task_1+path)-1) *2);
			//	break;
			}
			//case 1:
			else if(Lock_on_state == 1)
			{
				Lock_on_state = *(openmv_buf + (*(Rack_task_1+path)-1) *2);
				if(Lock_on_state == 0)
				{
					lock_v = 500;
					move[0] = 500;
					return;
				}
				if(Go == 0)
				{
					if(TX_SG)
					{
						SG_move_tx(11,500,SG_solo_tar[path]);//ת������
						TX_SG = 0;
					}
					
					Err = *(openmv_buf + (*(Rack_task_1+path)-1) *2 + 1) - 160;//��ƫ
					
					if(Err>-5 && Err<5)//����
					{
						lock_v = 0;
							Task_stop();
						Go = 1;//��ʼץȡ,��������
						TX_SG = 1;
						Beep(Be);
					}
					else//�ӽ�
					{
						Patrol_line_flag = 1;//����Ѳ��
						if(Err>0)
							lock_v = Err+10;
						else
							lock_v = Err-10;							
					}
				}
				else
				{
					Usart_tx_SG(1);//��ȡ
					HAL_Delay(4500);
					//HAL_Delay(4000);
					Lock_on_state = 0;
					Go = 0;
					path++;
					lock_v = 500;
					move[0] = 500;
					Beep(Be);
				}
				//break;
			}
		}
		move[0] = lock_v;
/*
		//״̬�ж�
		if(Lock_on_state == 0)//״̬0
		{
//			Lock_on_state = *(openmv_buf + (*(Rack_task_1+path)-1) *2);
			Patrol_line_flag = 1;//����Ѳ��
		}
		else if(Lock_on_state == 1)//״̬1����
		{
//			if(*(openmv_buf + (*(Rack_task_1+path)-1) *2))
//			{
				SG_move_tx(8,500,SG_solo_tar[path]);//ת������
				Err = *(openmv_buf + (*(Rack_task_1+path)-1) *2 + 1) - 160;//��ƫ
				
				if(Err>-5 && Err<5)//����
				{
					lock_v = 0;
					move[0] = 0;
					move[1] = 0;
					move[2] = 0;
					tube_adjust[0] = 0;
					tube_adjust[1] = 0;
					tube_adjust[2] = 0;
					Lock_on_state = 2;//��ʼץȡ,��������
				}
				else//�ӽ�
				{
					Patrol_line_flag = 1;//����Ѳ��
					if(Err>0)
						lock_v = Err+10;
					else
						lock_v = Err-10;							
				}
//			}
		}
		else//״̬2ִ�ж���
		{
			if(TX_state)
			{
				TX_state = 0;//��λ
				Lock_on_state = 0;
				path++;
				lock_v = 500;
			}
			else
			{
				Usart_tx_SG(1);//��ȡ
				TX_state = 1;
				HAL_Delay(4000);
				HAL_Delay(4000);
			}
		}
*/
	//}
}
//11
/**
 * @description: �������
 * @author: NiuTu
 * @param {*} 
 * @return {*}
 */
void Down_body(void)
{
	static uint8_t path = 0;

	Task_stop();
	//(*(Rack_task_1+path))
	if(path == 0)
	{
		for(uint8_t i = 0; i<3; i++)
		{
			SG_move_tx(11,250,SG_solo_tar[i]);//ת������
			HAL_Delay(260);
			Usart_tx_SG(24);
			HAL_Delay(2300);

			switch(*(Rack_task_1+i))
			{
				case 1:	Usart_tx_SG(14);break;
				case 2: Usart_tx_SG(13);break;
				case 3: Usart_tx_SG(12);break;
			}
			HAL_Delay(1805);
		}
	}
	else
	{
		for(uint8_t i = 0; i<3; i++)
		{
			SG_move_tx(11,250,SG_solo_tar[i]);//ת������
			HAL_Delay(260);
			Usart_tx_SG(24);
			HAL_Delay(2300);

			switch(*(Rack_task_2+i))
			{
				case 1:	Usart_tx_SG(14);break;
				case 2: Usart_tx_SG(13);break;
				case 3: Usart_tx_SG(12);break;
			}
			HAL_Delay(1805);
		}
	}
	path = 1;
	Task_Flag = free;//����
}

//12
/**
 * @description: �������
 * @author: NiuTu
 * @param {*} 
 * @return {*}
 */
void Up_body(void)
{
	static uint8_t path = 0;
	static uint8_t t = 0;
	
	//(*(Rack_task_1+path))
	if(path == 0)
	{
		if(t == 0)
		{
			for(uint8_t i = 0; i<3; i++)
			{
				SG_move_tx(11,250,SG_solo_tar[i]);//ת������
				HAL_Delay(260);
				switch(*(Rack_task_1+i))
				{
					
					case 1:	Usart_tx_SG(17);break;
					case 2: Usart_tx_SG(16);break;
					case 3: Usart_tx_SG(15);break;
				}
				HAL_Delay(2500);
				Usart_tx_SG(25);
				HAL_Delay(1625);
			}
			path = 1;
			
		}
		else
		{
			for(uint8_t i = 0; i<3; i++)
			{
				SG_move_tx(11,250,SG_solo_tar[i]);//ת������
				HAL_Delay(260);
				switch(*(Rack_task_2+i))
				{
					
					case 1:	Usart_tx_SG(17);break;
					case 2: Usart_tx_SG(16);break;
					case 3: Usart_tx_SG(15);break;
				}
				HAL_Delay(2500);
				Usart_tx_SG(25);
				HAL_Delay(1600);
			}
			path = 1;
		}
	}
	else if(path == 1)
	{
		Patrol_line_flag = 1;//����Ѳ��
		move[0] = -1000;
		move[1] = 0;
		move[2] = 0;
		if(B.scan_state == 1 && D.scan_state == 1)
		{ 
			move[0] = 0;
			path = 0;
			Task_Flag = free;//����
			t = 1;
		}
		//break;
	}
}

//13
/**
 * @description: ����������
 * @author: NiuTu
 * @param {*} 
 * @return {*}
 */
void Down_2_body(void)
{
	Task_stop();
	//(*(Rack_task_1+path))
	for(uint8_t i = 0; i<3; i++)
	{
		SG_move_tx(11,250,SG_solo_tar[i]);//ת������
		HAL_Delay(260);
		Usart_tx_SG(24);
		HAL_Delay(2300);
		switch(*(Rack_task_1+i))
		{
			case 1:	Usart_tx_SG(20);break;
			case 2: Usart_tx_SG(19);break;
			case 3: Usart_tx_SG(18);break;
		}
		HAL_Delay(2200);
	}
	HAL_Delay(500);
	Task_Flag = free;//����
}

//14
/**
 * @description: �����������
 * @author: NiuTu
 * @param {*} 
 * @return {*}
 */
void Down_3_body(void)
{
	Task_stop();
	//(*(Rack_task_1+path))

	for(uint8_t i = 0; i<3; i++)
	{
		SG_move_tx(11,250,SG_solo_tar[i]);//ת������
		HAL_Delay(260);
		Usart_tx_SG(24);
		HAL_Delay(2300);
		switch(*(Rack_task_2+i))
		{
			case 1:	Usart_tx_SG(23);break;
			case 2: Usart_tx_SG(22);break;
			case 3: Usart_tx_SG(21);break;
		}
		HAL_Delay(4000);
	}
	HAL_Delay(1000);
	Task_Flag = free;//����
}

//1
/**
 * @description: ��ת90��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void turn_L90(void)
{
	Patrol_line_flag = 3;
	if(Integral_move(0,0,-500,0,0,-80000))
	{
		Patrol_line_flag = 2;
		Task_Flag = free;
	}
} 

//16
/**
 * @description: ��ת90��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void turn_R90(void)
{
	Patrol_line_flag = 3;
	if(Integral_move(0,0,500,0,0,80000))
	{
		Patrol_line_flag = 2;
		Task_Flag = free;
	}	
} 

/**
 * @description: �������
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void SG_Dubug(void)
{
	static uint8_t TX_state = 0;//����ɹ�״̬
	if(!TX_state)
	{
		TX_state = 1;//����ɹ�
		SG_move_tx(8,500,675);
	}
	else
	{
		if(SG_rx_com == 0x08)
		{
			TX_state = 0;//��λ
			Task_Flag = free;
		}
	}
}
//18
/**
 * @description: һ��۲��ȡ
 * @author: NiuTu
 * @note: 012λ��д���ǻ�������λ�ü�012
 * @note: ���Rack_fact_1[]
 * @param {*}
 * @return {*}
 */
void First_get(void)
{
	static uint8_t path = 0;
	uint8_t i = 0;
	uint8_t L_Flag = 0;//������ȷ����־
	uint8_t M_Flag = 0;
	uint8_t R_Flag = 0;
	uint8_t Lose;//û��λ��ɫλ012RGB
	uint8_t Body_sum = 0;//��������������
	switch(path)
	{
		case 0:
		{
			Task_stop();
			Usart_tx_SG(1);//����һ���ϲ�۲�
			HAL_Delay(650);
			path = 1;
			Beep(BzzBe);
			break;
		}
		case 1:
		{
			Body_sum = *(openmv_buf+0) + *(openmv_buf+2) + *(openmv_buf+4);
			if(Body_sum >=2)
			{
				for(i = 0; i<3; i++)
				{
					if(*(openmv_buf+i*2))//������λ��
					{
						if(*(openmv_buf+i*2+1)>= 130 && *(openmv_buf+i*2+1)<= 190)
						{
							Rack_fact_1[i] = 1;
							M_Flag = 1;//��λ��־��1
							
						}
						else if(*(openmv_buf+i*2+1)<= 130)
						{
							Rack_fact_1[i] = 0;
							L_Flag = 1;//��λ��־��1
							
						}
						else if(*(openmv_buf+i*2+1)>= 190)
						{
							Rack_fact_1[i] = 2;
							R_Flag = 1;//��λ��־��1
							
						}
					}
					else
						Lose = i;
				}
				if(Body_sum ==2)
				{
					if(L_Flag == 0)
						Rack_fact_1[Lose] = 0;
					else if(M_Flag == 0)
						Rack_fact_1[Lose] = 1;
					else if(R_Flag == 0)
						Rack_fact_1[Lose] = 2;
				}

				Task_Flag = free;
				path = 0;
			}
			break;
		}
	}	
}
//19
/**
 * @description: ����۲��ȡ
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Second_get(void)
{
	static uint8_t path = 0;
	uint8_t i = 0;
	uint8_t L_Flag = 0;//������ȷ����־
	uint8_t M_Flag = 0;
	uint8_t R_Flag = 0;
	uint8_t Lose;//û��λ��ɫλ012RGB
	uint8_t Body_sum = 0;//��������������
		switch(path)
	{
		case 0:
		{
			Task_stop();
			Usart_tx_SG(2);//���������²�۲�
			HAL_Delay(250);
			path = 1;
			break;
		}
		case 1:
		{
			Body_sum = *(openmv_buf+0) + *(openmv_buf+2) + *(openmv_buf+4);
			if(Body_sum >=2)
			{
				for(i = 0; i<3; i++)
				{
					if(*(openmv_buf+i*2))//������λ��
					{
						if(*(openmv_buf+i*2+1)>= 130 && *(openmv_buf+i*2+1)<= 190)
						{
							Rack_fact_2[i] = 1;
							M_Flag = 1;//��λ��־��1
							
						}
						else if(*(openmv_buf+i*2+1)<= 130)//�������1�ų�û�ҵ������
						{
							Rack_fact_2[i] = 0;
							L_Flag = 1;//��λ��־��1
							
						}
						else if(*(openmv_buf+i*2+1)>= 190)
						{
							Rack_fact_2[i] = 2;
							R_Flag = 1;//��λ��־��1
							
						}
					}
					else
						Lose = i;
				}
				if(Body_sum ==2)
				{
					if(L_Flag == 0)
						Rack_fact_2[Lose] = 0;
					else if(M_Flag == 0)
						Rack_fact_2[Lose] = 1;
					else if(R_Flag == 0)
						Rack_fact_2[Lose] = 2;
				}

				Task_Flag = free;
			}
			break;
		}
	}	
}
//20
/**
 * @description: һ��ץȡ׼��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void First_prepare(void)
{
	Usart_tx_SG(3);//��������ץȡλ
	Task_Flag = free;
}

//22
/**
 * @description: һ��ץȡ
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void First_body_get(void)
{
	static uint8_t path = 0;
	switch(path)
	{
		case 0://��ǰ�˶������ѹ��
		{
			if(Move_to_line(0))
				path = 1;
			break;
		}
		case 1://ִ�м�ȡ
		{
			for(uint8_t i = 0; i < 3;i++)
			{
				SG_move_tx(11,250,SG_solo_tar[i]);//ת������
				HAL_Delay(260);
				Usart_tx_SG(*(Rack_fact_1 + *(Rack_task_1+i)-1)+ 5 );//567
				HAL_Delay(3700);

			}
			path = 2;
			break;
		}
		case 2://�����ص�
		{
			Patrol_line_flag = 1;//����Ѳ��
			move[0] = -600;
			move[1] = 0;
			move[2] = 0;
			if(B.scan_state == 1 && D.scan_state == 1)
			{ 
				Patrol_line_flag = 2;
				move[0] = 0;
				path = 0;
				Task_Flag = free;//����
			}
			break;
		}
	}
}
//23
/**
 * @description: ����ץȡ
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Second_body_get(void)
{
	static uint8_t order = 0;//orderҪ�����������ץȡ
	static uint8_t path = 0;
	switch (path)
	{

		case 0://��е��Ԥ��
		{
			Usart_tx_SG(*(Rack_fact_2 + *(Rack_task_2+order)-1)+ 8 );//8 9 10
			HAL_Delay(2000);
			SG_move_tx(11,250,SG_solo_tar[order]);//ת������
			path = 1;
			break;
		}
		case 1://��ǰѹ��
		{
			if(Move_to_line(0))
			{
				SG_move_tx(12,500,1210);//���Ӽн�
				HAL_Delay(500);
				path = 2;
			}	
			break;
		}
		case 2://����ѹ��
		{
			if(Move_to_line(2))
			{
				path = 3;
			}	
			break;
		}
		case 3://��е�۶���
		{
			Usart_tx_SG(11);//�ջ�
			HAL_Delay(2400);
			path = 0;
			order++;
			if(order == 3)
				path = 4;
			break;
		}
		case 4://��λ
		{
			Patrol_line_flag = 1;//����Ѳ��
			move[0] = 600;
			move[1] = 0;
			move[2] = 0;
			if(B.scan_state == 1 && D.scan_state == 1)
			{ 
				path = 0;
				Task_Flag = free;//����
				Patrol_line_flag = 2;
				move[0] = 0;
			}
			break;
		}
	}
}
//24
/**
 * @description:�ڽ����׼ȷ����λ��
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void max_adjust(void)
{
	move[0] = 0;
	move[1] = 0;
	move[2] = 0;
	Patrol_line_flag = 2;  
	HAL_Delay(1000);
	Patrol_line_flag = 3;  
	Task_Flag = free;//����
}



