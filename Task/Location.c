/*
 * @Author: niutu
 * @Date: 2021-02-03 18:04:12
 * @LastEditTime: 2021-04-13 20:47:45
 * @LastEditors: NiuTu
 * @Description:位置确定
 * @FilePath: \MDK-ARMd:\project\logistics_robot_v1\Task\Location.c
 */
#include "Location.h"
#include "Bsp.h"
//宏定义


//全局变量
/*
    A
D       B
    C
*///四组数组abcd
uint8_t Gray_level_A[4]; 
uint8_t Gray_level_B[4]; 
uint8_t Gray_level_C[4]; 
uint8_t Gray_level_D[4]; 

//结构体声明
GurytubeTypeDef A,B,C,D;//四个传感器结构体
//函数
/**
 * @description: 传感器结构体初始化
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Location_line_init(void)
{
    A.ID = 1;//填写id
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
 * @description: 更新灰度的状态到数组
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
 * @description: 寻线函数整合
 * @author: NiuTu
 * @param {*}
 * @return {*}
 */
void Location_line(void)
{
    GuryTube_state(Gray_level_A,&A);
    GuryTube_state(Gray_level_B,&B);
    GuryTube_state(Gray_level_C,&C);
    GuryTube_state(Gray_level_D,&D);//状态填写
    /*
    Pressure_Line(&A);
    Pressure_Line(&B);
    Pressure_Line(&C);
    Pressure_Line(&D);//压线计数
    */
    ScanLine(&A);
    ScanLine(&B);
    ScanLine(&C);
    ScanLine(&D);//扫线判断
}


/**
 * @description: 状态填写
 * @author: NiuTu
 * @param {uint8_t} *Gray_level
 * @param {GurytubeTypeDef} *id
 * @return {*}
 */
uint8_t GuryTube_state(uint8_t *Gray_level,GurytubeTypeDef *id )
{
    //运动方向状态传递为传感器相对方向
    switch (id->ID)//选择传感器
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
    //传感器状态更新
    if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 0)//传感器悬空XXXX
    {
        id->state = 0;//状态编号0
        id->Pressure_Line_state = 0;//压线状态更新
    }
    else if(*(Gray_level + 0) == 1 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 0)//传感器扫线OXXX
    {
        id->state = 1;//状态编号1
        id->Pressure_Line_state = 0;//压线状态更新
        id->Last_on_state = 1;//上轮触发状态更新
    }
    else if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 1 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 0)//传感器扫线XOXX
    {
        id->state = 2;//状态编号2
        id->Pressure_Line_state = 0;//压线状态更新
        id->Last_on_state = 2;//上轮触发状态更新
    }
    else if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 1 && *(Gray_level + 3) == 0)//传感器扫线XXOX
    {
        id->state = 3;//状态编号3
        id->Pressure_Line_state = 0;//压线状态更新
        id->Last_on_state = 3;//上轮触发状态更新
    }
    else if(*(Gray_level + 0) == 0 && *(Gray_level + 1) == 0 
    && *(Gray_level + 2) == 0 && *(Gray_level + 3) == 1)//传感器扫线XXXO
    {
        id->state = 4;//状态编号4
        id->Pressure_Line_state = 0;//压线状态更新
        id->Last_on_state = 4;//上轮触发状态更新
    }
    else if(*(Gray_level + 0) == 1 && *(Gray_level + 3) == 1)//两端高电，传感器压线,OOOO
    {
        id->state = 5;//状态编号5
        id->Pressure_Line_state = 1;//压线状态更新
    }
    else//不予参考
    {
        id->state = 6;//状态编号6
    }
    //方向数据的更新
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
        //方向数据的更新
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
 * @description: 压线计数
 * @author: NiuTu
 * @param {GurytubeTypeDef} *id 传感器id，选B,D
 * @return {*}
 */
void Pressure_Line(GurytubeTypeDef *id)
{
    if(id->state == 6)//不进行判断
    {
        return;
    }
    if(!id->Last_Pressure_Line_state == id->Pressure_Line_state)//压线状态跳变
    {
        if(id->Pressure_Line_state == 1)  //跳变为上线
        {
            id->scan_num_Y = id->scan_num_Y + id->dive_dir_Y;//压线计数
            id->Last_dir_Y = id->dive_dir_Y;//记录上线方向
            
        }
        else                //跳变为下线
        {
            if(id->Last_dir_Y != id->dive_dir_Y)//下线方向与上线方向不同，更新计数
            {
                id->scan_num_Y = id->scan_num_Y + id->dive_dir_Y;//压线计数
            }
        }
        id->Last_Pressure_Line_state = id->Pressure_Line_state;//更新压线状态
    }   
}

/**
 * @description: 扫线判断
 * @author: NiuTu
 * @param {GurytubeTypeDef} *id 传感器id
 * @return {*}
 */
void ScanLine(GurytubeTypeDef *id)
{
    /*筛去不判断情况*/
    if(id->state == 6 )
    {
        return;//不进行判断
    }
    if(id->state == 5)
    {
		//id->err = 0;//不执行校准数据
        return;//压线没有判断的必要
    }
    
    if(id->Last_state != id->state) //扫线状态跳变
    {
        if(id->state != 0)              //跳变为上线
        {
            if(id->scan_state == 0)     //离线到扫线
            {
                if(id->state == 1)//左上线
                {
                    id->scan_state = 1;//标记为扫线在线
                    id->scan_on_dir = -1;//初始化扫线方向
                    id->err = -4;
                    if(id->scan_off_dir == id->scan_on_dir)//离线方向与上线方向同向
                    {
                        id->scan_num_X--;//左触发减去扫线数
                    }
                }
                else if(id->state == 4)//右上线
                {
                    id->scan_state = 1;//标记为扫线在线
                    id->scan_on_dir =  1;//初始化扫线方向，在离线的时候判断是否增减
                    id->err =  4;
                    if(id->scan_off_dir == id->scan_on_dir)//离线方向与上线方向同向
                    {
                        id->scan_num_X++;//右触发增加扫线数
                    }     
                }
                else//中间上线是压线的前状态，结束
                return;
            }
            else                        //扫线情况下的上线
            {
                /*巡线状态填写*/
                switch(id->state)
                {
                case 1://极限救回
                    id->err = -4;
                    break;
                case 2://偏向传感器的左
                    id->err = -1;
                    break;
                case 3://偏向传感器的右
                    id->err =  1;
                    break;            
                case 4://极限救回
                    id->err =  4;
                    break;
                }
            }
        }
        else                            //跳变为下线
        {
            //2,3对管向中间下线
            if((id->Last_state == 2 && id->dive_dir_X == 1)||(id->Last_state == 3 && id->dive_dir_X == -1))
            {
                id->err = 0;//巡线不偏
            }
			if(id->Last_state == 1 && id->dive_dir_X == 1 )//左离线
            {
                id->scan_state = 0;//标记为离线
                id->scan_off_dir = 1;//更新离线方向
//                    if(id->scan_off_dir != id->scan_on_dir)//离线方向与上线方向反向
//                    {
//                        id->scan_num_X++;//左触发减去扫线数
//                    }
            }
            else if(id->Last_state == 4 && id->dive_dir_X ==-1)//右离线
            {
                id->scan_state = 0;//标记为扫线在线
                id->scan_off_dir = -1;//更新离线方向，在离线的时候判断是否增减
//                    if(id->scan_off_dir != id->scan_on_dir)//离线方向与上线方向反向
//                    {
//                        id->scan_num_X--;//右触发增加扫线数
//                    }     
            }
//            else if(((id->Last_state == 4 && id->dive_dir_X ==-1)
//                ||(id->Last_state == 1 && id->dive_dir_X == 1 ))
//                && id->num_update_flag)//左右离开传感器且没有在巡线
//            {
//                id->scan_state = 0;//离线
//                id->scan_off_dir = id->dive_dir_X;//更新离线方向
//            }
            //无端下线，默认为外丢向内纠正快速得到新的纠正结果
            
            else if(id->dive_dir_X == 0)
            {
                switch(id->Last_state)
                {
                case 1://极限救回
                    id->err = -4;
                    break;
                case 2://偏向传感器的左
                    id->err = -1;
                    break;
                case 3://偏向传感器的右
                    id->err =  1;
                    break;            
                case 4://极限救回
                    id->err =  4;
                    break;
                }
            }
        }
    id->Last_state = id->state;//更新上次状态
    } 
}
/*
        if(id->err !=0)
        {
			if(id->num_update_flag)//没有在巡线
			{
				if(id->scan_on_dir != id->err) //偏差方向跳
				{
					
				}
				id->scan_on_dir = id->err;//更新上次偏差
			}
        }
        */



