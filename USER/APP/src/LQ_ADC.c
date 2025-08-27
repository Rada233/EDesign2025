
#include "include.h"

ADC_HandleTypeDef ADC1_Handler;		//ADC句柄
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void ADC_init(void)
@功能说明：ADC初始化
@参数说明：ch: ADC_channels 
@函数返回：无
@修改时间：2022/02/24
@调用方法：ADC_InitConfig();
@备    注：通道值 0~16取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void ADC_Init(void)
{ 
	RCC_PeriphCLKInitTypeDef ADC_CLKInit;
	
	ADC_CLKInit.PeriphClockSelection=RCC_PERIPHCLK_ADC;		//ADC外设时钟
	ADC_CLKInit.AdcClockSelection=RCC_ADCPCLK2_DIV6;			//分频因子6时钟为72M/6=12MHz
	HAL_RCCEx_PeriphCLKConfig(&ADC_CLKInit);							//设置ADC时钟
	
	ADC1_Handler.Instance=ADC1;
	ADC1_Handler.Init.DataAlign=ADC_DATAALIGN_RIGHT;             //右对齐
	ADC1_Handler.Init.ScanConvMode=DISABLE;                      //非扫描模式
	ADC1_Handler.Init.ContinuousConvMode=DISABLE;                //关闭连续转换
	ADC1_Handler.Init.NbrOfConversion=1;                         //1个转换在规则序列中 也就是只转换规则序列1 
	ADC1_Handler.Init.DiscontinuousConvMode=DISABLE;             //禁止不连续采样模式
	ADC1_Handler.Init.NbrOfDiscConversion=0;                     //不连续采样通道数为0
	ADC1_Handler.Init.ExternalTrigConv=ADC_SOFTWARE_START;       //软件触发
	HAL_ADC_Init(&ADC1_Handler);                                 //初始化 
	
	HAL_ADCEx_Calibration_Start(&ADC1_Handler);					 //校准ADC
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
@功能说明：ADC底层驱动，引脚配置，时钟使能
@参数说明：hadc:ADC句柄
@函数返回：
@修改时间：2022/02/24
@调用方法：
@备    注：此函数会被HAL_ADC_Init()调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_Initure;
    __HAL_RCC_ADC1_CLK_ENABLE();            //使能ADC1时钟
    __HAL_RCC_GPIOA_CLK_ENABLE();			//开启GPIOA时钟
		__HAL_RCC_GPIOB_CLK_ENABLE();			//开启GPIOA时钟
    GPIO_Initure.Pin=GPIO_PIN_2 | GPIO_PIN_3;            //PA1
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	  GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1;            //PA1
    GPIO_Initure.Mode=GPIO_MODE_ANALOG;     //模拟
    GPIO_Initure.Pull=GPIO_NOPULL;          //不带上下拉
    HAL_GPIO_Init(GPIOB,&GPIO_Initure);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：uint16_t Get_Adc(uint32_t ch)   
@功能说明：获取ADC转换数据
@参数说明：ch: 通道值 0~16，取值范围为：ADC_CHANNEL_0~ADC_CHANNEL_16
@函数返回：ADC读到的值
@修改时间：2022/02/24
@调用方法：vbat=ADC_Read(ADC0);
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
uint16_t Get_Adc(uint32_t ch)   
{
    ADC_ChannelConfTypeDef ADC1_ChanConf;   
    ADC1_ChanConf.Channel=ch;                                   //通道
    ADC1_ChanConf.Rank=1;                                       //第1个序列，序列1
    ADC1_ChanConf.SamplingTime=ADC_SAMPLETIME_239CYCLES_5;      //采样时间               
    HAL_ADC_ConfigChannel(&ADC1_Handler,&ADC1_ChanConf);        //通道配置
    HAL_ADC_Start(&ADC1_Handler);                               //开启ADC
    HAL_ADC_PollForConversion(&ADC1_Handler,10);                //轮询转换
 
	return (u16)HAL_ADC_GetValue(&ADC1_Handler);	        	//返回最近一次ADC1规则组的转换结果
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned short ADC_Read(ADC_Channel_t  chn)
@功能说明：ADC多次采集取平均值
@参数说明：chn    : ADC通道
@参数说明：times  : 采集次数
@函数返回：ADC数值
@修改时间：2020年3月10日
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

uint16_t Get_Adc_Average(uint32_t ch,uint8_t times)
{
	uint32_t temp_val=0;
	uint8_t t;
	for(t=0;t<times;t++)
	{
		temp_val+=Get_Adc(ch);
		delay_ms(5);
	}
	return temp_val/times;
} 

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned short ADC_Read(ADC_Channel_t  chn)
@功能说明：得到温度值
@参数说明：val   : ADC读取值
@函数返回：温度值
@修改时间：2020年3月10日
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
//
//返回值:温度值(扩大了100倍,单位:℃.)
int Get_Temp(uint32_t val)
{
	int Temp;
 	double temperate;
	
	temperate=(float)val*(3.3/4096);				//电压值 
	temperate=(1.43-temperate)/0.0043+25;			//转换为温度值 	 
	Temp=temperate*100;							//扩大100倍.
	return Temp;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Test_ADC(void)
@功能说明：ADC测试函数
@参数说明：无
@函数返回：无
@修改时间：2020年3月10日
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_ADC(void)
{
	char txt[32];
	uint32_t val1;
	int Temp;
	uint32_t Val4, Val5, Val6, Val7;
	OLED_Init();   	//OLED初始化
	OLED_CLS(); 		//清屏
	ADC_Init();			//ADC初始化

	while(1)
	{
		//val1 = Get_Adc(1);
		Val4 = Get_Adc(2);
		Val5 = Get_Adc(3);
		Val6 = Get_Adc(8);
		Val7 = Get_Adc(9);
		Temp = Get_Temp(val1);
		
		printf("Temp:%04d ADC4:%04d ADC5:%04d ADC6:%04d ADC7:%04d\n",val1, Val4, Val5, Val6, Val7);
		sprintf(txt,"ADC4:%04d",Val4);
		OLED_P6x8Str(0,2,txt);
		sprintf(txt,"ADC5:%04d",Val5);
		OLED_P6x8Str(0,3,txt);
		sprintf(txt,"ADC6:%04d",Val6);
		OLED_P6x8Str(0,4,txt);
		sprintf(txt,"ADC7:%04d",Val7);
		OLED_P6x8Str(0,5,txt);
		
		sprintf(txt,"temp:%03d.%02d",Temp/100,Temp%100);
		OLED_P6x8Str(0,7,txt);
		delay_ms(100); 
	}
}



