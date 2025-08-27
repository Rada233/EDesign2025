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
@函数名称：KEY_Init
@功能说明：按键初始化
@参数说明：无
@函数返回：无
@修改时间：2022/2/19
@调用方法：KEY_Init();
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();                       //开启GPIOA时钟
		__HAL_RCC_GPIOB_CLK_ENABLE();                   //开启GPIOB时钟

    GPIO_Initure.Pin=GPIO_PIN_11 | GPIO_PIN_12;  	    //PA11  12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);    
	
		GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;  //PB0  1  2
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;            //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：Read_key
@功能说明：判断相关按键是否按下
@参数说明：num :按键编号
@函数返回：状态 1：按下 0：没按下
@修改时间：2022/2/19
@调用方法：if(Read_key(KEY0)) KEY0是否按下
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
u8 Read_key(key_num num)
{
	switch(num)
	{
		case 1:
			if(HAL_GPIO_ReadPin(KEY1_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY1_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 2:
			if(HAL_GPIO_ReadPin(KEY2_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY2_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 3:
			if(HAL_GPIO_ReadPin(KEY3_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY3_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 4:
			if(HAL_GPIO_ReadPin(KEY4_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY4_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
			
		case 5:
			if(HAL_GPIO_ReadPin(KEY5_Pin) == GPIO_PIN_RESET)
			{
				delay_ms(10);
				if(HAL_GPIO_ReadPin(KEY5_Pin) == GPIO_PIN_RESET)
					return 1;
			}
			break;
	}
	return 0;
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：Test_key
@功能说明：按键功能测试函数
@参数说明：无
@函数返回：无
@修改时间：2022/2/19
@调用方法：Test_key();
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_key(void)
{
	char txt[32];
	uart_init(USART_2, 115200);			//初始化串口
	LED_Init();							//初始化LED	
	KEY_Init();							//初始化按键
	OLED_Init();						//初始化屏幕
	
	sprintf(txt, "KEY TEST");
	OLED_P8x16Str(40, 0, txt);
	sprintf(txt, "The KEY is");
	while(1)
	{
		if(Read_key(KEY1))
		{
			sprintf(txt, "The KEY is KEY1");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY2))
		{
			sprintf(txt, "The KEY is KEY2");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY3))
		{
			sprintf(txt, "The KEY is KEY3");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY4))
		{
			sprintf(txt, "The KEY is KEY4");
			LED_Ctrl(RVS);
		}
		if(Read_key(KEY5))
		{
			sprintf(txt, "The KEY is KEY5");
			LED_Ctrl(RVS);
		}
		
		OLED_P8x16Str(0, 2, txt);
	}
}






