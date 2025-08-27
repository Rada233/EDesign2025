/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"


//uint16_t BatVolt;         // 电池电压采集

//int16_t MotorDuty1 = 500;      // 电机驱动占空比数值
//int16_t MotorDuty2 = 500;      // 电机驱动占空比数值
//int32_t NowTime = 0;
//uint16_t BatVolt = 0;           // 电池电压采集


//#define MOTOR_FREQUENCY    PWM_DUTY_MAX

//#define USE7843or7971   //USEDRV8701 使用龙邱不同的驱动模块，选择对应的宏定义
#define USEDRV8701

TIM_HandleTypeDef 	TIM1_Handler;
TIM_OC_InitTypeDef  TIM1_CH1Handler;    		//定时器1通道1句柄
TIM_IC_InitTypeDef  TIM1_CH4Handler;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TIM1_PWM_Init(u16 arr,u16 psc)
@功能说明：PWM初始化
@参数说明：arr：重装载值，psc：预分频值
@函数返回：无
@修改时间：2022/03/15
@调用方法：	TIM1_PWM_Init(7199, 0);  10K Hz
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM1_PWM_Init(u16 arr,u16 psc)
{
//    TIM_HandleTypeDef 	TIM1_Handler;
//    TIM_OC_InitTypeDef  TIM1_CH1Handler;    		//定时器1通道1句柄
    
	TIM1_Handler.Instance=TIM1;             		//定时器1
	TIM1_Handler.Init.Prescaler=psc;           	    //定时器分频
	TIM1_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
	TIM1_Handler.Init.Period=arr;              	    //自动重装载值
	TIM1_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_Base_Init(&TIM1_Handler);
 
	HAL_TIM_PWM_Init(&TIM1_Handler);           	             //初始化PWM
	HAL_TIM_Base_Start_IT(&TIM1_Handler);                    //使能定时器3和定时器3更新中断：TIM_IT_UPDATE  

	TIM1_CH1Handler.OCMode=TIM_OCMODE_PWM1; 		//模式选择PWM1
	TIM1_CH1Handler.Pulse=	0;            			//设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
	TIM1_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高  

	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_1);//配置TIM1通道1    
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_2);//配置TIM1通道2   
	HAL_TIM_PWM_ConfigChannel(&TIM1_Handler,&TIM1_CH1Handler,TIM_CHANNEL_3);//配置TIM1通道3 
	
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_1);//开启PWM通道1
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_2);//开启PWM通道2
	HAL_TIM_PWM_Start(&TIM1_Handler,TIM_CHANNEL_3);//开启PWM通道3
	

    HAL_TIM_PWM_Start_IT(&TIM1_Handler, TIM_CHANNEL_1);  //启动主定时器PWM输出

	TIM1_CH4Handler.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
	TIM1_CH4Handler.ICSelection = TIM_ICSELECTION_DIRECTTI;
	TIM1_CH4Handler.ICPrescaler = TIM_ICPSC_DIV1;
	TIM1_CH4Handler.ICFilter = 0;
	HAL_TIM_IC_ConfigChannel(&TIM1_Handler, &TIM1_CH4Handler, TIM_CHANNEL_4);
}
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base)
{
  if(htim_base->Instance==TIM1)
  {
    __HAL_RCC_TIM1_CLK_ENABLE();
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void MotorInit(void)
@功能说明：电机初始化
@参数说明：无
@函数返回：无
@修改时间：2022/03/15
@调用方法：MotorInit();
@备    注：驱动电机
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void MotorInit(void)
{
	GPIO_InitTypeDef 		GPIO_Initure;
	
	__HAL_RCC_TIM1_CLK_ENABLE();                //使能定时器1
	__HAL_RCC_GPIOA_CLK_ENABLE();               //开启GPIOA时钟  
	__HAL_RCC_GPIOB_CLK_ENABLE();               //开启GPIOB时钟

	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;      //PA8 9 10
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 

	GPIO_Initure.Pin=GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;    //PB12 13 14
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;          //输出
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	TIM1_PWM_Init(7199, 0);
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TIM1_PWM_Init(u16 arr,u16 psc)
@功能说明：PWM初始化
@参数说明：arr：重装载值，psc：预分频值，duty，占空比
@函数返回：无
@修改时间：2022/03/15
@调用方法：	TIM4_PWM_Init(7199, 0,duty);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TIM4_PWM_Init(u16 arr,u16 psc, u16 duty)
{
    TIM_HandleTypeDef 	TIM4_Handler;
	TIM_OC_InitTypeDef  TIM4_CH1Handler;    		//定时器4通道1句柄
	TIM4_Handler.Instance=TIM4;             		//定时器1
	TIM4_Handler.Init.Prescaler=psc;           	//定时器分频
	TIM4_Handler.Init.CounterMode=TIM_COUNTERMODE_UP;//向上计数模式
	TIM4_Handler.Init.Period=arr;              	//自动重装载值
	TIM4_Handler.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
	HAL_TIM_PWM_Init(&TIM4_Handler);           	//初始化PWM
	HAL_TIM_Base_Start_IT(&TIM4_Handler); 			//使能定时器4更新中断：TIM_IT_UPDATE  

	TIM4_CH1Handler.OCMode=TIM_OCMODE_PWM1; 			//模式选择PWM1
	TIM4_CH1Handler.Pulse=	duty;            			//设置比较值,此值用来确定占空比，默认比较值为自动重装载值的一半,即占空比为50%
	TIM4_CH1Handler.OCPolarity=TIM_OCPOLARITY_HIGH; //输出比较极性为高  
	
	HAL_TIM_PWM_ConfigChannel(&TIM4_Handler,&TIM4_CH1Handler,TIM_CHANNEL_1);//配置TIM4通道1 
	HAL_TIM_PWM_Start(&TIM4_Handler,TIM_CHANNEL_1);//开启PWM通道1
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void ServoInit(u16 freq)
@功能说明：舵机初始化
@参数说明：无
@函数返回：无
@修改时间：2022/03/15
@调用方法：ServoInit(100);
@备    注：主函数调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ServoInit(u16 duty, u16 psc)
{
	GPIO_InitTypeDef  GPIO_Initure;
	__HAL_RCC_TIM4_CLK_ENABLE();            //使能定时器4
	__HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟

	GPIO_Initure.Pin=GPIO_PIN_6;      			//PB6
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;      //复用推挽输出
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
	if(psc == 100)							 //100Hz
		TIM4_PWM_Init(9865, 72, duty);
	else	                                  //50Hz
		TIM4_PWM_Init(9999, 144, duty);
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Brushless_Motor_Init(u16 freq)
@功能说明：无刷电机PWM初始化
@参数说明：freq：初始化频率
@函数返回：无
@修改时间：2024/06/11
@调用方法：	Brushless_Motor_Init(50)；
@备    注：使用无刷电机驱动的时候尽量不要使用有刷相关函数，二者输出管脚相同，以免冲突
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Brushless_Motor_Init(u16 freq)
{
    GPIO_InitTypeDef  GPIO_Initure;
	__HAL_RCC_TIM1_CLK_ENABLE();            //使能定时器
	__HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIO时钟

	
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;      //PA8 9 10
	GPIO_Initure.Mode=GPIO_MODE_AF_PP;              //复用推挽输出
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;		//高速
	HAL_GPIO_Init(GPIOA,&GPIO_Initure); 
    if(freq == 100)
		TIM1_PWM_Init(9865, 72);          //100Hz
	else	                                 
		TIM1_PWM_Init(9999, 144);         //50Hz
}



//-----------------------------------------------------------------------------------

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void MotorCtrl3w(int16_t motor1, int16_t motor2,int16_t motor3)
@功能说明：电机控制函数
@参数说明：motor1:电机1占空比，范围【-10000，10000】
@函数返回：无
@修改时间：2022/02/24
@调用方法：MotorCtrl3w(pid1.out, pid2.out, pid3.out);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void MotorCtrl3w(int16_t motor1, int16_t motor2, int16_t motor3)
{  
//		设置TIM1通道1的占空比
//		compare:比较值
//    TIM1->CCR1=compare; 
  if (motor1 > 0)
  {
    TIM1->CCR1=motor1;	    //C6
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_RESET);   //G5	
  }
  else
  {
    TIM1->CCR1=0-motor1;
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_SET);
  }
  
 if (motor2 > 0)
  {
    TIM1->CCR2=motor2;	 //C7
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_RESET);//G6	
  }
  else
  {
    TIM1->CCR2=0-motor2;
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_SET);
  }
  
	if (motor3 > 0)
  {
    TIM1->CCR3=motor3;	     //C8
    HAL_GPIO_WritePin(MTGPIO3p, GPIO_PIN_RESET);    //G7
  }
  else
  {
    TIM1->CCR3=0-motor3;
    HAL_GPIO_WritePin(MTGPIO3p, GPIO_PIN_SET);
  }
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void MotorCtrl(int16_t motor1, int16_t motor2)
@功能说明：双路电机驱动函数
@参数说明：motor1:电机1占空比，范围【-10000，10000】
@函数返回：无
@修改时间：2022/02/24
@调用方法：MotorCtrl(pid1.out, pid2.out);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

void MotorCtrl(int16_t motor1, int16_t motor2)
{
	if (motor1 > 0)
  {
    TIM1->CCR1=motor1;	    //C6
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_RESET);   //G5	
  }
  else
  {
    TIM1->CCR1=0-motor1;
    HAL_GPIO_WritePin(MTGPIO1p, GPIO_PIN_SET);
  }
  
 if (motor2 > 0)
  {
    TIM1->CCR2=motor2;	 //C7
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_RESET);//G6	
  }
  else
  {
    TIM1->CCR2=0-motor2;
    HAL_GPIO_WritePin(MTGPIO2p, GPIO_PIN_SET);
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void ServoCtrl (uint16_t duty)
@功能说明：舵机控制函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：ServoCtrl(750);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ServoCtrl(int16_t motor1)
{
    if (motor1 > 0)
    {
        TIM4->CCR1=motor1;	     //B6
    }
    else
    {
        TIM4->CCR1=0-motor1;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Brushless_Motor_Ctrl(int16_t motor1, int16_t motor2, int16_t motor3)
@功能说明：无刷电机控制函数
@参数说明：motor1:电机1占空比，范围
@函数返回：无
@修改时间：2024/06/11
@调用方法：MotorCtrl3w(pid1.out, pid2.out, pid3.out);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Brushless_Motor_Ctrl(int16_t motor1, int16_t motor2, int16_t motor3)
{  
//		设置TIM1通道1的占空比,可以先限幅
//		compare:比较值
//      TIM1->CCR1=compare; 

  if (motor1 > 0)
  {
    TIM1->CCR1=motor1;	      //PA8
  }
  
 if (motor2 > 0)
  {
    TIM1->CCR2=motor2;	      //PA9
  }
  
 if (motor3 > 0)
  {
    TIM1->CCR3=motor3;	      //PA10
  }

}


//-----------------------------------------------------------------------------------

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TestMotor (void)
@功能说明：电机测试函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：TestMotor();
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TestMotor (void)
{
    int16_t duty = 1000,flag = 0;
    char txt[16];
	OLED_Init();            	//OLED初始化
	OLED_CLS(); 
    MotorInit();
	sprintf(txt, "Test Motor");
	OLED_P6x8Str(30,0,txt);
  while (1)
  {
    if(Read_key(KEY1) == 1)      //按下KEY0键，占空比减小
    {
//      if(duty > -PWM_DUTY_MAX)
        duty += 500;
    }
    if(Read_key(KEY2) == 1)      //按下KEY2键，占空比加大
    {
//      if(duty < PWM_DUTY_MAX)   //满占空比为10000
        duty = 0;
    }
    if(Read_key(KEY3) == 1)      //按下KEY1键，占空比中值
    {
      duty -= 500;
    }
    
    
    /*  自动加减速测试 */   //240814
    if(duty >= 3500)//6000
        flag = 1;
    if(duty <= -3500)
        flag = 0;
    
    if(flag == 0)
        duty += 100;
    else
        duty -= 100;
    
    MotorCtrl3w(duty,duty, duty);
//	ServoCtrl(duty);
    sprintf(txt, "PWM: %5d;", duty);
	OLED_P6x8Str(5,7,txt);
    
    LED_Ctrl(RVS);       //电平翻转,LED闪烁
    delay_ms(100);       //延时等待
  }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void TestServo (void)
@功能说明：舵机测试函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：TestServo();
@备    注：
@注意事项:注意!!!!!!!!!!!!!!!!!!一定要对舵机打角进行限制!!!!!!!!!!!!!!
使用龙邱母板测试流程：
1.先使用万用表测量电池电压，务必保证电池电压在7V以上，否则无力不反应！
2.然后确定舵机供电电压，SD5舵机用5V供电，S3010用6-7V供电！！！
3.把舵机的舵盘去掉，让舵机可以自由转动；
4.烧写程序并运行，让舵机转动到中值附近；如果没反应重复1-2步，或者调整舵机的PWM频率计占空比，能受控为准；
5.舵机受控后用手轻转，舵机会吱吱响，对抗转动，此时可以装上舵盘；
6.按键K0/K1确定舵机的左右转动极限，并记下来，作为后续限幅防止舵机堵转烧毁！
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void TestServo (void)
{
	int16_t duty = 0;
  char txt[16];
  
	OLED_Init();            	//OLED初始化
	OLED_CLS(); 
    ServoInit(0, 10);
	sprintf(txt, "Test Servo");
	OLED_P6x8Str(30,0,txt);
  while (1)
  {
    if(Read_key(KEY1) == 1)      //按下KEY1键，占空比减小
    {
        duty += 10;
    }
    if(Read_key(KEY2) == 1)      //按下KEY2键，占空比加大
    {
      duty = 530;
    }
    if(Read_key(KEY3) == 1)      //按下KEY3键，占空比中值
    {
        duty -= 10;
    }
		//限幅
//		if(duty>100) duty=100;
//		if(duty<-100) duty=-100;
		
//	ServoCtrl(Servo_Center_Mid+duty);
    ServoCtrl(duty);
    sprintf(txt, "PWM: %5d;", duty);
	OLED_P6x8Str(5,3,txt);
    
    LED_Ctrl(RVS);       //电平翻转,LED闪烁
    delay_ms(200);              //延时等待
  }
}



/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Test_Brushless_Motor(void)
@功能说明：无刷电机测试函数，占用原来的三路电机接
@参数说明：无
@函数返回：无
@修改时间：2024/06/11
@调用方法：Test_Brushless_Motor()；
@备    注：占用原来的8701驱动三路直流有刷电机接
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_Brushless_Motor(void)
{
    int16_t duty = 0;
    char txt[16];
  
	OLED_Init();            	//OLED初始化
	OLED_CLS(); 
    Brushless_Motor_Init(50);   //50Hz
    Brushless_Motor_Ctrl(duty, duty, duty);
	sprintf(txt, "Test Brushless Motor");
	OLED_P6x8Str(0,0,txt);
  while (1)
  {
    if(Read_key(KEY1) == 1)      //按下KEY0键，占空比减小
    {
        duty += 20;
    }
    if(Read_key(KEY2) == 1)      //按下KEY2键，占空比加大
    {
      duty = 530;
    }
    if(Read_key(KEY3) == 1)      //按下KEY1键，占空比中值
    {
        duty -= 20;
    }
    Brushless_Motor_Ctrl(duty, duty, duty);
    sprintf(txt, "PWM: %5d;", duty);
	OLED_P6x8Str(2,7,txt);
    
    LED_Ctrl(RVS);       //电平翻转,LED闪烁
    delay_ms(200);       //延时等待
  }
}

