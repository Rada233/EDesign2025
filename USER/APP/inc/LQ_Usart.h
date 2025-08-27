#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 
//********************************************************************************

#define USART_REC_LEN  			200  		//定义最大接收字节数 200
#define EN_USART3_RX 			1			//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; 	//接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
extern u16 USART_RX_STA;         			//接收状态标记	
extern UART_HandleTypeDef UART3_Handler; 	//UART句柄

#define RXBUFFERSIZE   1 					//缓存大小
extern u8 aRxBuffer[RXBUFFERSIZE];			//HAL库USART接收Buffer

extern u8 receive_oneline2;		//串口2接收标志
extern u8 receive_oneline3;    //串口3接收标志

typedef enum		// 枚举串口号
{
  USART_2 = 2,
	USART_3,
}USART_NUM;


void uart_SendBuf(UART_HandleTypeDef *huart, uint8_t *pData);
void uart_init(USART_NUM usart,u32 bound);
void Test_Usart(void);
void BlueTooth_cont(void);

#endif


