/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技F3277核心板和母板
【编    写】龙邱科技
【E-mail  】chiusir@163.com
【软件版本】V1.0 版权所有，单位使用请先联系授权
【最后更新】2020年12月24日，持续更新，请关注最新版！
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【IDE】IAR7.8 KEIL5.24及以上版本
【Target 】 MM32F3277
【SYS PLL】 120MHz 频率太高可能无法启动system_mm32f327x.c
=================================================================
程序配套视频地址：https://space.bilibili.com/95313236

硬件使用SPI1
CS   PE10/---------9++++++++++++++++++++999
SCK  PE12
MISO PE13 
MOSI PE14

#define Soft_SPI_CS_PIN       GPIOE,GPIO_Pin_10     //CS 默认是PE11,如果使用编码器，则可以切换到PE10
#define Soft_SPI_SCK_PIN      GPIOE,GPIO_Pin_12     //SCK
#define Soft_SPI_MISO_PIN     GPIOE,GPIO_Pin_13     //单片机MISO接模块的--SDO
#define Soft_SPI_MOSI_PIN     GPIOE,GPIO_Pin_14     //单片机MOSI接模块的--SDI

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"

////////////////////////////////////////////////////////////////////////////////

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】 void Soft_SPI_Init(void)
【功  能】 模拟SPI初始化
【参数值】 无
【参数值】 无
【返回值】 无 
【作  者】 L Q
【最后更新】 2021年1月22日 
【软件版本】 V1.0
【调用样例】 Soft_SPI_Init()
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Soft_SPI_Init(void)
{  
	GPIO_InitTypeDef GPIO_Initure;

	__HAL_RCC_GPIOA_CLK_ENABLE();           	//开启GPIOB时钟
    
	//PB8/9初始化设置
	GPIO_Initure.Pin=GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
	GPIO_Initure.Pin=GPIO_PIN_6;
	GPIO_Initure.Mode=GPIO_MODE_INPUT;  		//浮空输入
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
	HAL_GPIO_Init(GPIOA,&GPIO_Initure);
	
  
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】 void Soft_SPI_ReadWriteNbyte(u8 *lqbuff, u8 len)
【功  能】 模拟SPI读写数据及长度
【参数值】 u8 *buf数据指针,u16 len长度
【参数值】 u8 *buf数据指针,u16 len长度
【返回值】 无 
【作  者】 L Q
【最后更新】 2021年1月22日 
【软件版本】 V1.0
【调用样例】 SPI_Read_Nbytes(SPI1,reg|0x80,buf,len);
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Soft_SPI_ReadWriteNbyte(u8 *lqbuff, u8 len)
{
  u8 i;
  
  Soft_SPI_CS_OUT_L;
  Soft_SPI_SCK_OUT_H;
  do
  {
    for(i = 0; i < 8; i++)
    {
      if((*lqbuff) >= 0x80)
      {
        Soft_SPI_MOSI_OUT_H;        
      }
      else
      {
        Soft_SPI_MOSI_OUT_L;       
      }        
      
      Soft_SPI_SCK_OUT_L;
      (*lqbuff) <<= 1;      
      
      Soft_SPI_SCK_OUT_H;
      
      (*lqbuff) |= Soft_SPI_MISO_IN;          
    }
    lqbuff++;
  }while(--len);
  Soft_SPI_CS_OUT_H;
}


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】void LQ_SPI_Read(unsigned char reg, unsigned short len, unsigned char* buf)
【功  能】SPI从设备读取数据
【参数值】SPI_TypeDef* SPIx, SPI通道选择
【参数值】unsigned char reg,设备起始地址
【参数值】unsigned char* buf,数据存放地址
【参数值】unsigned short len,待读取的数据长度
【返回值】无 
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】SPI_Read_Nbytes(SPI1,reg|0x80,buf,len);
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_SPI_Read(unsigned char reg, unsigned short len, unsigned char* buf)
{  
  unsigned short i;
  buf[0] = reg | 0x80;  
  Soft_SPI_ReadWriteNbyte(buf, len + 1); 
  for(i=0;i<len;i++)
    buf[i]=buf[i+1];
}
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【函数名】void LQ_SPI_Write(unsigned char reg, unsigned short len, unsigned char* buf)
【功  能】SPI向设备写入数据
【参数值】
【参数值】unsigned char reg,设备起始地址
【参数值】unsigned char* buf,数据存放地址
【参数值】unsigned short len,待写入的数据长度
【返回值】无 
【作  者】L Q
【最后更新】2021年1月22日 
【软件版本】V1.0
【调用样例】SPI_Read_Nbytes(SPI1,reg|0x80,buf,len);
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void LQ_SPI_Write(unsigned char reg, unsigned short len, unsigned char* buf)
{   

  unsigned short i;
  for(i=0;i<len;i++)
    buf[len-i]=buf[len-i-1];
  buf[0] = reg | 0x80;  
  
  Soft_SPI_ReadWriteNbyte(buf, len + 1);  

}

