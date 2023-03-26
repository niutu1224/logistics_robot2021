<!--
 * @Author: NiuTu
 * @Date: 2023-03-16 12:20:13
 * @LastEditors: NiuTu
 * @FilePath: \logistics_robot_end\README.md
-->
# logistics_robot2021
## 项目介绍
&emsp;&emsp;这个仓库是我大学参加2021年工程训练哈尔滨赛区智能搬运组的全部设计的归档，包含PCB设计，机械设计，程序等。  
&emsp;&emsp;本项目使用STM32F103ZET6芯片作为主控，采用cubemx生成工程。PCB资料放在立创EDA开源里（还没过审）小车底盘采用麦克纳姆轮构型，用灰度传感器巡线确定位置，机械手因为时间原因放弃了使用步进电机的方案，改用了串口通信的舵机控制板。