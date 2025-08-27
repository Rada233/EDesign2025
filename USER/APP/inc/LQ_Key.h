/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编    写：龙邱科技
@E-mail  ：chiusir@163.com
@编译IDE：KEIL5.25.3及以上版本
@使用平台：北京龙邱智能科技全向福来轮小车
@最后更新：2022年02月19日，持续更新，请关注最新版！
@功能介绍：
@相关信息参考下列地址
@网    站：http://www.lqist.cn
@淘宝店铺：http://longqiu.taobao.com
@软件版本：V1.0 版权所有，单位使用请先联系授权
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _KEY_H
#define _KEY_H
#include "sys.h"



//按键引脚宏定义
#define KEY1_Pin        GPIOB,GPIO_PIN_0  	//KEY0按键PB0
#define KEY2_Pin        GPIOB,GPIO_PIN_1  	//KEY1按键PB1
#define KEY3_Pin       	GPIOB,GPIO_PIN_2  	//KEY2按键PB2
#define KEY4_Pin        GPIOA,GPIO_PIN_11  	//KEY3按键PA11
#define KEY5_Pin        GPIOA,GPIO_PIN_12  	//KEY4按键PA12

//按键选择
typedef enum
{
  KEY1=1,
	KEY2,
	KEY3,
	KEY4,
	KEY5
}key_num;

void KEY_Init(void);				//按键初始化
u8 Read_key(key_num num);		//读取按键状态
void Test_key(void);				//按键功能测试

#endif


