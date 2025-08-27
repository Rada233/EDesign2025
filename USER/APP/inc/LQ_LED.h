/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@编    写：龙邱科技
@E-mail  ：chiusir@163.com
@编译IDE：KEIL5.25.3及以上版本
@使用平台：北京龙邱智能科技全向福来轮小车
@最后更新：2022年02月19日，持续更新，请关注最新版！
@功能介绍：LED
@相关信息参考下列地址
@网    站：http://www.lqist.cn
@淘宝店铺：http://longqiu.taobao.com
@软件版本：V1.0 版权所有，单位使用请先联系授权
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#ifndef _LED_H
#define _LED_H
#include "sys.h"

#define led GPIOB,GPIO_PIN_15		//LED管脚定义


//枚举LED状态
typedef enum
{
  ON=0,  //亮
  OFF=1, //灭
  RVS=2, //反转
}LEDs_e;

#define LED0 PBout(15)   	//LED0

void LED_Init(void);
void LED_Ctrl(LEDs_e sta);
void Test_Led(void);

#endif


