#include "include.h"


TIM_HandleTypeDef 	TIM_Handler;
TIM_Encoder_InitTypeDef  Encoder_sConfig;

int ECPULSE1 = 0,ECPULSE2 = 0,ECPULSE3 = 0;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Encoder_Init_TIM2(void)
@功能说明：编码器1初始化
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：Encoder_Init_TIM2();
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Encoder_Init_TIM2(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;
	__HAL_RCC_TIM2_CLK_ENABLE();            	//使能定时器2
	__HAL_RCC_GPIOA_CLK_ENABLE();            	//开启GPIOA时钟
	__HAL_AFIO_REMAP_TIM2_PARTIAL_1();          //外设复用1
	
	GPIO_Initure.Pin=GPIO_PIN_0 |GPIO_PIN_1;    //PA0 1
	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //输入模式
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;	 //高速
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 
    
//	__HAL_RCC_GPIOB_CLK_ENABLE();            	//开启GPIOB时钟	
//	GPIO_Initure.Pin= GPIO_PIN_3;  //PA0 1
//	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //复用输入
//	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;	//高速
//	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	
	TIM_Handler.Instance=TIM2;             		//定时器1
	TIM_Handler.Init.Prescaler=0;           	//定时器分频
	TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
	TIM_Handler.Init.Period=ENCODER_TIM_PERIOD;     //自动重装载值
	TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//不分频
	TIM_Handler.Init.RepetitionCounter = 0;//重复计数器
	TIM_Handler.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
    Encoder_sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    Encoder_sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_sConfig.IC1Filter = 0;
    Encoder_sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	Encoder_sConfig.IC2Filter = 0;
	
	HAL_TIM_Encoder_Init(&TIM_Handler, &Encoder_sConfig);

    HAL_TIM_Encoder_Start(&TIM_Handler, TIM_CHANNEL_ALL);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Encoder_Init_TIM3(void)
@功能说明：编码器2初始化
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：Encoder_Init_TIM3();
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Encoder_Init_TIM3(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;

	__HAL_RCC_TIM3_CLK_ENABLE();                //使能定时器3
	__HAL_RCC_GPIOB_CLK_ENABLE();               //开启GPIOB时钟
    __HAL_AFIO_REMAP_TIM3_PARTIAL();            //使能复用重映射,PB4,B5

	
	GPIO_Initure.Pin=GPIO_PIN_4| GPIO_PIN_5;    //PB4 5
	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //输入模式
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	
	TIM_Handler.Instance=TIM3;             		//定时器3
	TIM_Handler.Init.Prescaler=0;           	//定时器分频
	TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数模式
	TIM_Handler.Init.Period=ENCODER_TIM_PERIOD;             //自动重装载值
	TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//不分频
	TIM_Handler.Init.RepetitionCounter = 0;                 //重复计数器
	TIM_Handler.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
    Encoder_sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    Encoder_sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_sConfig.IC1Filter = 0;
    Encoder_sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	Encoder_sConfig.IC2Filter = 0;
	
	HAL_TIM_Encoder_Init(&TIM_Handler, &Encoder_sConfig);

    HAL_TIM_Encoder_Start(&TIM_Handler, TIM_CHANNEL_ALL);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Encoder_Init_TIM4(void)
@功能说明：编码器3初始化
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：Encoder_Init_TIM4();
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Encoder_Init_TIM4(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;
	__HAL_RCC_TIM4_CLK_ENABLE();                //使能定时器4
	__HAL_RCC_GPIOB_CLK_ENABLE();               //开启GPIOB时钟
	
	GPIO_Initure.Pin=GPIO_PIN_6 | GPIO_PIN_7;   //PB6 7
	GPIO_Initure.Mode=GPIO_MODE_INPUT;          //输入模式
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;    //高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	
	TIM_Handler.Instance=TIM4;             		//定时器4
	TIM_Handler.Init.Prescaler=0;           	//定时器分频
	TIM_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;        //向上计数模式
	TIM_Handler.Init.Period=ENCODER_TIM_PERIOD;             //自动重装载值
	TIM_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;	//不分频
	TIM_Handler.Init.RepetitionCounter = 0;                 //重复计数器
	TIM_Handler.Init.AutoReloadPreload  = TIM_AUTORELOAD_PRELOAD_ENABLE;
	
    Encoder_sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
    Encoder_sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    Encoder_sConfig.IC1Filter = 0;
    Encoder_sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    Encoder_sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    Encoder_sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	Encoder_sConfig.IC2Filter = 0;
	
	HAL_TIM_Encoder_Init(&TIM_Handler, &Encoder_sConfig);

    HAL_TIM_Encoder_Start(&TIM_Handler, TIM_CHANNEL_ALL);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int Read_Encoder(u8 TIMX)
@功能说明：读取编码器计数
@参数说明：u8 TIMX定时器
@函数返回：脉冲数量
@修改时间：2022/02/24
@备    注：ECPULSE1 = Read_Encoder(3); //左电机 母板上编码器3，小车前进为负值
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int Read_Encoder(u8 TIMX)
{
    int Encoder_TIM;    
   switch(TIMX)
	 {
        case 2:  Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;
         break;
        case 3:  Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;
         break;	
		case 4:  Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;
         break;	
		default:  Encoder_TIM=0;
	 }
		return -Encoder_TIM;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TIM4_IRQHandler(void)
@功能说明：定时器中断服务函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：无
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM4_IRQHandler(void)
{ 		    		  			    
	if(TIM4->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM4->SR&=~(1<<0);//清除中断标志位 	    
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TIM3_IRQHandler(void)
@功能说明：定时器中断服务函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：无
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM3_IRQHandler(void)
{ 		    		  	
	HAL_TIM_IRQHandler(&TIM3_Handler);		    
	if(TIM3->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM3->SR&=~(1<<0);//清除中断标志位 	    
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TIM2_IRQHandler(void)
@功能说明：定时器中断服务函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：无
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM2_IRQHandler(void)
{ 		    		  	
	HAL_TIM_IRQHandler(&TIM2_Handler);		    
	if(TIM2->SR&0X0001)//溢出中断
	{    				   				     	    	
	}				   
	TIM2->SR&=~(1<<0);//清除中断标志位 	    
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Test_ENC(void)
@功能说明：测试程序，TFT1.8显示
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_ENC(void)
{
	
	char txt[30];
	Encoder_Init_TIM2();
	Encoder_Init_TIM3();
	Encoder_Init_TIM4();
	
	while(1)
	{
//		enc = (uint32_t)(__HAL_TIM_GET_COUNTER(&TIM_Handler));
		
		ECPULSE1=Read_Encoder(2);
		sprintf(txt,"E1:%04d ",ECPULSE1);
	    OLED_P6x8Str(0,2,txt);	
		ECPULSE2=Read_Encoder(3);
		sprintf(txt,"E2:%04d ",ECPULSE2);
	    OLED_P6x8Str(0,3,txt);	
		ECPULSE3=Read_Encoder(4);
		sprintf(txt,"E3:%04d ",ECPULSE3);
	    OLED_P6x8Str(0,4,txt);	
        
        delay_ms(100);   //计数 累计时间
		
	}
}
