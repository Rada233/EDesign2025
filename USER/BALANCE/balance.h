/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技自平衡摩托车
【编    写】chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.1 
【最后更新】2020年4月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Keil5.2及以上版本
【Target 】 STM32F103
【Crystal】 8.000Mhz
【SYS PLL】 72MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __CONTROL_H
#define __CONTROL_H
#include "include.h"

#define PI 3.14159265

#define DIFFERENCE 160
#define LQSKH   1       //1 科宇车模 

#ifdef LQSKH            //科宇车模  		
			#define SERVO_MID 754  //舵机的中间位置，左转930--780--630右边
#else			
			#define SERVO_MID 750
#endif

//int EXTI9_5_IRQHandler(void);
int Balance(float Angle,float Gyro);
int velocity(int encoder_left,int encoder_right);
int turn(int encoder_left,int encoder_right,float gyro);
int turn(int encoder_left,int encoder_right,float gyro);
void Key(void);
void Xianfu_Pwm(void);
int myabs(int a);
void OLED_show(void);
extern short Encoder_A,Encoder_B,Encoder_C,Encoder_D;   //编码器的脉冲计数
extern int Left ,Right; 
void balance_car(void);
void Infra_Init(void);
void Infra_car(void);
#endif
