#include "LQ_HCSR04.h"
#include "include.h"

/*
1.接线:
	  超声波模块与开发板用杜邦线相连，板子PB9接Trig, PB8接Echo, GND接GND 5V接VCC。
2.实验:
	  下载程序，全速运行，安装OLED屏幕，屏幕显示距离值。说明实验成功
*/
/*-----------------------------------------------------------
定时器初始化 初始化定时器2 定时时间为10us 用来给超声波计时，系统主频72MHz

-----------------------------------------------------------*/
/**************************************************
 * 函数名: TIM2_Init(u16 arr,u16 psc)
 * 功  能: 定时器2初始化参数配置
 * 返回值: 装载值arr，预分频系数值psc
 * /中断服务函数在 stm32f1xx_it.c中
 ***************************************************/
 
TIM_HandleTypeDef TIM2_Handler;      //定时器句柄 

void LQ_TIM2_Init(u16 arr,u16 psc)
{  
    __HAL_RCC_TIM2_CLK_ENABLE();                            //使能TIM3时钟
    TIM2_Handler.Instance=TIM2;                             //通用定时器3
    TIM2_Handler.Init.Prescaler=psc;                        //分频系数
    TIM2_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;       //向上计数器
    TIM2_Handler.Init.Period=arr;                           //自动装载值
    TIM2_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1; //时钟分频因子
    HAL_TIM_Base_Init(&TIM2_Handler);
    
    HAL_TIM_Base_Start_IT(&TIM2_Handler);   //使能定时器3和定时器2更新中断：TIM_IT_UPDATE   
	HAL_NVIC_SetPriority(TIM2_IRQn,0,1);    //设置中断优先级，抢占优先级1，子优先级3
	HAL_NVIC_EnableIRQ(TIM2_IRQn);          //开启ITM2中断  
}

void Ultrasonic_Init()
{
	//初始化 超声波模块引脚
    GPIO_InitTypeDef GPIO_Initure;
  
	__HAL_RCC_GPIOB_CLK_ENABLE();                       //开启GPIOB时钟

    GPIO_Initure.Pin=GPIO_PIN_9;  	                    //PB9 // Trig - PB9输出
    GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;      	    //输出
    GPIO_Initure.Pull=GPIO_PULLUP;        			    //上拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);    
	
	GPIO_Initure.Pin=GPIO_PIN_8;                       //PB8 // Echo - PB8输
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;            //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);
//	LQ_TIM2_Init(71,9);                    // 定时器用于计数
}
extern uint32_t TimeCounter;	 //用于超声波计时,该变量位于stm32f1xx_it.c中
uint32_t HalTime1=0, HalTime2=0;
uint32_t Get_Distance(void) // 传入时间单位10us
{
	uint32_t Distance = 0;
	/*将Trig引脚拉高十微秒*/
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
	delay_us(10);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	//如果Echo引脚是低电平则一直等待，直到为Echo引脚为高电平，为高电平就说明接受到了返回的信号
    HalTime1= TimeCounter;
	while(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 0)
    {
        if(TimeCounter - HalTime1 > 2200)  break;  //10us *2000      
    }
	//记录下此时的时间值（10us定时器中累加的变量，所以时间单位为10us）
//    printf("TIMEA:%d \r\n", TimeCounter - HalTime1); // 打印正常读取数据时的值大小
	HalTime1 = TimeCounter;
	while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8) == 1)	//等待高电平时间结束
    {
        if(TimeCounter - HalTime1 > 4500)  break;
    }
//    printf("TIMEB:%d \r\n", TimeCounter - HalTime1); // 打印正常读取数据时的值大小
	//计算时间 判断时间变量是不是从头开始累加了（变量超过范围时会从0开始累加）
	if (TimeCounter > HalTime1)
	{
		//计算高电平的时间长度
		HalTime2 = TimeCounter - HalTime1;
//        Distance=(uint32_t)((float)HalTime2/58*10);//距离单位cm,声速340M/S，时间*速度/2=距离
        Distance = (uint32_t)(((float)HalTime2 *17)/100);

	}
	if( 2 < Distance)
        return Distance;
    else 
        return 100;
}

void Test_Ultrasonic()
{
	uint16_t Dis = 0.0;
	char txt[32];
	Ultrasonic_Init();
	OLED_Init();						 // OLED初始化
	OLED_P6x8Str(10, 0, "Test HCSR04 "); // 字符串
	delay_ms(5);
	while (1)
	{
		Dis = Get_Distance();
		sprintf(txt, "Dis=%3d cm", Dis);
		OLED_P8x16Str(10, 2, txt); // 显示字符串
	
		LED_Ctrl(RVS);
		delay_ms(100);
	}
}
