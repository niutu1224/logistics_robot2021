<!--
 * @Author: NiuTu
 * @Date: 2023-03-16 12:20:13
 * @LastEditors: NiuTu
 * @FilePath: \logistics_robot_end\README.md
-->
# logistics_robot2021
## 项目介绍
&emsp;&emsp;这个仓库是我大学参加2021年工程训练哈尔滨赛区智能搬运组的全部设计的归档，包含PCB设计，机械设计，程序等。  
&emsp;&emsp;本项目使用STM32F103ZET6芯片作为主控，采用cubemx生成工程。PCB资料放在立创EDA开源里[（点击这里）](https://oshwhub.com/gong-xun-4/stm32f103zet6_mainboard)  
 &emsp;&emsp;小车底盘采用麦克纳姆轮构型，用灰度传感器巡线确定位置，机械手因为时间原因放弃了使用步进电机的方案，改用了串口通信的舵机控制板。  
 ### 文件结构介绍  
 #### Bsp
 Bsp是板级支持包，里面是用户写的各种外设的使用。  
 Bsp.c用于所有外设的初始化  
 Beep_bsp.c嗡鸣器的驱动文件  
 IRQHandler_bsp.c中断服务相关  
 Stepmotor_bsp.c步进电机驱动文件（本工程最后没有用到）  
 Usart_bsp.c串口相关驱动文件
 #### Src与Inc
 这是cubemx生成单片机资源配置文件。其中stm32f1xx_it.c中写有串口接收回调函数（详见Usart_bsp.c）
 #### LCD
 2.2寸spi通信FTF屏幕的驱动包，由于有多个文件为了便于分辨没有放在Bsp文件夹中。完整的驱动包可以去LCD wiki查找下载。其中的lcd_task是用户编写的。
 #### MDK-ARM
 cubemx生成的文件，存放mdk工程
 #### Task
 存放了所有的用于控制运动的代码，几乎是除中断服务函数之外最核心的代码  
 Control_Arm.c机械臂控制写了简单的解算，由于时间不足更改了方案，没有使用。  
 Control_Logic.c包含了机器人巡线，各种运动方式，任务包等。我将完成比赛拆分成一个个任务包，每一个任务包可以独立执行，以便于调试和复用。
 Location.c灰度传感器的数据处理，对所有触发结果进行归类，并通过各种结果先后状态排列来判断灰度传感器与地面网格线的相对运动关系（本应该做到外力作用下推动机器人在网格线上平移即可得到，但采用的灰度传感器只有4路对管不利于一些情况判断，所以引入了底盘控制的值作为运动方向的判断依据）
 #### logistics_robot_end.ioc
 cubemx的配置文件


