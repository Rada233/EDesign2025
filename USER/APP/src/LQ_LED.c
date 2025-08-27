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
#include "include.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：LED_Init
@功能说明：LED灯初始化
@参数说明：无
@函数返回：无
@修改时间：2022/2/19
@调用方法：LED_Init()
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Init(void)
{
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOB_CLK_ENABLE();           	//开启GPIOB时钟

	GPIO_Initure.Pin=GPIO_PIN_15; 			  //PB15
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  	//推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          	//上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
	
	HAL_GPIO_WritePin(led,GPIO_PIN_RESET);		//PB15置1，默认初始化后灯灭
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：LED_Ctrl
@功能说明：控制LED状态
@参数说明：sta:状态 ON打开 OFF关闭 RVS反转
@函数返回：无
@修改时间：2022/2/19
@调用方法：LED_Ctrl(ON);
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LED_Ctrl(LEDs_e sta)
{
	if(sta==ON)        HAL_GPIO_WritePin(led, GPIO_PIN_RESET);
	else if(sta==OFF)  HAL_GPIO_WritePin(led, GPIO_PIN_SET);
	else if(sta==RVS)  HAL_GPIO_WritePin(led, (HAL_GPIO_ReadPin(led)?GPIO_PIN_RESET:GPIO_PIN_SET));
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：Test_Led
@功能说明：测试LED
@参数说明：无
@函数返回：无
@修改时间：2022/2/19
@调用方法：Test_Led()
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_Led(void)
{
  LED_Init();
  OLED_P6x8Str(0,1,"LQ_LED Test");         //字符串
  while(1)
  {
    LED_Ctrl(RVS); 
    delay_ms(200);
  }
}


