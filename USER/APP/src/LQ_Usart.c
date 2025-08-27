	 
#include "include.h"

//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 到串口2
int fputc(int ch, FILE *f)
{      
	while((USART2->SR&0X40)==0);//循环发送,直到发送完毕   
  USART2->DR = (u8) ch;      
	return ch;
}
#endif 



//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	
u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  

u8 aRxBuffer[RXBUFFERSIZE];//HAL库使用的串口接收缓冲
UART_HandleTypeDef USART2_Handler; //USART2句柄
UART_HandleTypeDef USART3_Handler; //USART3句柄

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void uart_init(USART_NUM usart,u32 bound)		
@功能说明：串口初始化
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：无
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void uart_init(USART_NUM usart,u32 bound)				//初始化串口
{
	switch(usart)
	{
		case 2:
			USART2_Handler.Instance=USART2;                         //USART2
			USART2_Handler.Init.BaudRate=bound;	                    //波特率
			USART2_Handler.Init.WordLength=UART_WORDLENGTH_8B;      //字长为8位数据格式
			USART2_Handler.Init.StopBits=UART_STOPBITS_1;           //一个停止位
			USART2_Handler.Init.Parity=UART_PARITY_NONE;            //无奇偶校验位
			USART2_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;      //无硬件流控
			USART2_Handler.Init.Mode=UART_MODE_TX_RX;               //收发模式
			HAL_UART_Init(&USART2_Handler);                         //HAL_UART_Init()会使能UART3
			HAL_UART_Receive_IT(&USART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
		
		case 3:
			USART3_Handler.Instance=USART3;                         //USART3
			USART3_Handler.Init.BaudRate=bound;                     //波特率
			USART3_Handler.Init.WordLength=UART_WORDLENGTH_8B;      //字长为8位数据格式
			USART3_Handler.Init.StopBits=UART_STOPBITS_1;	    	//一个停止位
			USART3_Handler.Init.Parity=UART_PARITY_NONE;            //无奇偶校验位
			USART3_Handler.Init.HwFlowCtl=UART_HWCONTROL_NONE;      //无硬件流控
			USART3_Handler.Init.Mode=UART_MODE_TX_RX;               //收发模式
			HAL_UART_Init(&USART3_Handler);	                        //HAL_UART_Init()会使能UART3
			HAL_UART_Receive_IT(&USART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
		break;
	}
}

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    //GPIO端口设置
	GPIO_InitTypeDef GPIO_Initure;
	
	if(huart->Instance==USART2)//如果是串口3，进行串口3 MSP初始化
	{
		__HAL_RCC_GPIOA_CLK_ENABLE();					//使能GPIOA时钟
		__HAL_RCC_USART2_CLK_ENABLE();				//使能USART3时钟
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_2;					//PA2
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;				//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	   	//初始化A2

		GPIO_Initure.Pin=GPIO_PIN_3;					//PA3
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//模式要设置为复用输入模式！	
		HAL_GPIO_Init(GPIOA,&GPIO_Initure);	  //初始化A3
		

		HAL_NVIC_EnableIRQ(USART2_IRQn);			//使能USART2中断通道
		HAL_NVIC_SetPriority(USART2_IRQn,3,3);//抢占优先级3，子优先级3

	}
	
	if(huart->Instance==USART3)//如果是串口3，进行串口3 MSP初始化
	{
		__HAL_RCC_GPIOB_CLK_ENABLE();			//使能GPIOA时钟
		__HAL_RCC_USART3_CLK_ENABLE();			//使能USART3时钟
		__HAL_RCC_AFIO_CLK_ENABLE();
	
		GPIO_Initure.Pin=GPIO_PIN_10;			//PB10
		GPIO_Initure.Mode=GPIO_MODE_AF_PP;		//复用推挽输出
		GPIO_Initure.Pull=GPIO_PULLUP;			//上拉
		GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB10

		GPIO_Initure.Pin=GPIO_PIN_11;			//PB11
		GPIO_Initure.Mode=GPIO_MODE_AF_INPUT;	//模式要设置为复用输入模式！	
		HAL_GPIO_Init(GPIOB,&GPIO_Initure);	   	//初始化PB11

		HAL_NVIC_EnableIRQ(USART3_IRQn);				//使能USART3中断通道
		HAL_NVIC_SetPriority(USART3_IRQn,3,2);			//抢占优先级3，子优先级2

	}
}

u8 ReadBuff[1024];
u16 num2=0, num3=0;
u8 receive_oneline2=0;
u8 receive_oneline3=0;
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
@功能说明：串口回调函数
@参数说明：*huart (串口句柄)
@函数返回：无
@修改时间：2022/02/24
@调用方法：无
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART2)//如果是串口2
	{
		if(HAL_UART_Receive_IT(&USART2_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) == HAL_OK)//接收成功
		{
			if(aRxBuffer[0] == '\n')// || aRxBuffer[0] == '\r')  //以回车为结束符
			{
				
				ReadBuff[num2] = '\0';
				num2=0;
                receive_oneline2 = 1;
			}else{
				ReadBuff[num2++] = aRxBuffer[0];
			}
		}else{
			printf("receive faild\n");
		}
	}
	if(huart->Instance==USART3)//如果是串口3
	{
		if(HAL_UART_Receive_IT(&USART3_Handler, (u8 *)aRxBuffer, RXBUFFERSIZE) == HAL_OK)//接收成功
		{

//            uart_SendBuf(&USART3_Handler, aRxBuffer);
            
			if(aRxBuffer[0] == '\n')// || aRxBuffer[0] == '\r')//以回车为结束符
			{
				ReadBuff[num3] = '\0'; //，结尾加“\0”
                num3=0;
                receive_oneline3 = 1;  //接收完成标志 1
				
			}else{
				ReadBuff[num3++] = aRxBuffer[0];
			}
            
            
		}else{
			printf("receive faild\n");
		}

	}
}
 
//串口2中断服务程序
void USART2_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&USART2_Handler);	//调用HAL库中断处理公用函数-----该函数会清空中断标志，取消中断使能，并间接调用回调函数
} 




//串口3中断服务程序
void USART3_IRQHandler(void)                	
{ 
	HAL_UART_IRQHandler(&USART3_Handler);	//调用HAL库中断处理公用函数-----该函数会清空中断标志，取消中断使能，并间接调用回调函数
} 


void uart_SendBuf(UART_HandleTypeDef *huart, uint8_t *pData)
{
//	while(len--)
//	{
		if(huart->Instance==USART2)
		{
			HAL_UART_Transmit(&USART2_Handler, pData, strlen((const char*)pData), 100);
		}
		if(huart->Instance==USART3)
		{
			HAL_UART_Transmit(&USART3_Handler, pData, strlen((const char*)pData), 100);
		}
	//}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Test_Usart(void)
@功能说明：蓝牙模块测试函数(串口3)
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：Test_Usart3()
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_Usart(void)
{	
	uart_init(USART_3, 115200);			//初始化串口    
	uart_init(USART_2, 115200);			//初始化串口    蓝牙接口
    OLED_Init();
	LED_Init();							//初始化LED	
	printf("Long Qiu Ke Ji\n");
	printf("USART3 test,This is USART2.\n");
    uart_SendBuf(&USART3_Handler, "This is USART3!\n");
    OLED_P6x8Str(0,3,"Long Qiu Ke Ji");         //字符串
    OLED_P6x8Str(0,5,"UART Test");              //字符串
	while(1)
	{
		if(receive_oneline2 != 0)//一行接收成功
		{
			printf("··receive %d byte data is %s\n",receive_oneline2, ReadBuff);
			receive_oneline2=0;
		}
		if(receive_oneline3 != 0)
		{
			uart_SendBuf(&USART3_Handler, (uint8_t*)ReadBuff);
			receive_oneline3=0;
		}
		delay_ms(200);   
		LED_Ctrl(RVS); 
	}
}

