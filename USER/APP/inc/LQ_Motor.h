/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技F3277核心板和母板
【编    写】龙邱科技
【E-mail  】chiusir@163.com
【软件版本】V1.0 版权所有，单位使用请先联系授权
【最后更新】2020年12月24日，持续更新，请关注最新版！
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【IDE】IAR7.8 KEIL5.24及以上版本
【Target 】 MM32F3277
【SYS PLL】 120MHz 频率太高可能无法启动system_mm32f327x.c
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef __LQ_MOTOR_H
#define __LQ_MOTOR_H

#include "include.h"

//#define Servo_Center_Mid 730        //舵机直行中值，
//#define MtTargetDuty  3300          //电机占空比值得最大值，实际需要减去电池电压，编码器，赛道曲率，达不到该值

#define	PWM_DUTY_MAX 7200 

#define MTGPIO1p      GPIOB,GPIO_PIN_12        //电机驱动方管脚
#define MTGPIO2p      GPIOB,GPIO_PIN_13  
#define MTGPIO3p      GPIOB,GPIO_PIN_14 

//extern int16_t ServoDuty;
//extern int16_t MotorDuty1;          // 电机驱动占空比数值
//extern int16_t MotorDuty2;          // 电机驱动占空比数值

//extern uint16_t BatVolt;            // 电池电压采集


void MotorInit(void);
void TIM1_PWM_Init(u16 arr,u16 psc);
void MotorCtrl(int16_t motor1, int16_t motor2);
void MotorCtrl3w(int16_t motor1, int16_t motor2, int16_t motor3);
void Brushless_Motor_Ctrl(int16_t motor1, int16_t motor2, int16_t motor3);
void Brushless_Motor_Init(u16 freq);

void ServoInit(u16 duty, u16 psc);
void ServoCtrl(int16_t motor1);
void TIM4_PWM_Init(u16 arr,u16 psc, u16 duty);

void TestMotor(void);
void TestServo(void);
void Test_Brushless_Motor(void);


#endif /* SRC_APPSW_TRICORE_USER_LQ_MOTORCONTROL_H_ */
