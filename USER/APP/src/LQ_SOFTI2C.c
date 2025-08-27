/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL

QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include <stdint.h>
#include "LQ_SOFTI2C.h"

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void SOFT_IIC_delay(u16 us)
@功能说明：IIC延时
@参数说明：us：延时时间
@函数返回：无
@修改时间：2022/02/24
@调用方法：SOFT_IIC_delay();  //简单的延时
@备    注：修改这里可以调整IIC速率
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_delay(u16 us)
{
  delay_us(us);  
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void IIC_Init(void)
@功能说明：模拟IIC初始化
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：IIC_Init();  //模拟IIC初始化
@备    注：IIC管脚在LQ_SOFTI2C.h中定义
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Init(void)
{			
	GPIO_InitTypeDef GPIO_Initure;
	__HAL_RCC_GPIOB_CLK_ENABLE();           	//开启GPIOB时钟   
	//PB8/9初始化设置
	GPIO_Initure.Pin=GPIO_PIN_8|GPIO_PIN_9;
	GPIO_Initure.Mode=GPIO_MODE_OUTPUT_PP;  //推挽输出
	GPIO_Initure.Pull=GPIO_PULLUP;          //上拉
	GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;//高速
	HAL_GPIO_Init(GPIOB,&GPIO_Initure);
    IIC_SCL_H;
    IIC_SDA_H;
    SOFT_IIC_Stop();
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void SOFT_IIC_Start(void)
@功能说明：模拟IIC起始信号
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：IIC_Start();
@备    注：内部调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Start(void)
{
  IIC_SCL_L;
  SDA_OUT;   //sda线输出 
  IIC_SDA_H;	
  IIC_SCL_H;
  SOFT_IIC_delay(1);
  IIC_SDA_L; //START:when CLK is high,DATA change form high to low 
  SOFT_IIC_delay(1);
  IIC_SCL_L; //钳住I2C总线，准备发送或接收数据 
  SOFT_IIC_delay(1);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void SOFT_IIC_Stop(void)
@功能说明：模拟IIC停止信号
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：IIC_Stop();
@备    注：内部调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Stop(void)
{  
  IIC_SCL_L;
  SDA_OUT; //sda线输出
  IIC_SDA_L; //STOP:when CLK is high DATA change form low to high
  SOFT_IIC_delay(1);
  IIC_SCL_H; 
  SOFT_IIC_delay(1);
  IIC_SDA_H; //发送I2C总线结束信号
  SOFT_IIC_delay(1);							   	
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void IIC_NAck(void)
@功能说明：模拟IIC不产生ACK应答
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：IIC_NAck();
@备    注：内部调用 主机接收完最后一个字节数据后，主机产生的NACK通知从机发送结束，
@           释放SDA,以便主机产生停止信号
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Ack(void)
{
  IIC_SCL_L;
  SDA_OUT;
  IIC_SDA_L;
  SOFT_IIC_delay(1);
  IIC_SCL_H;
  SOFT_IIC_delay(2);
  IIC_SCL_L;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void SOFT_IIC_NAck(void)
@功能说明：模拟IIC产生ACK应答
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@调用方法：SOFT_IIC_NAck();
@备    注：内部调用 主机接收完一个字节数据后，主机产生的ACK通知从机一个字节数据已正确接收
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_NAck(void)
{
  IIC_SCL_L;
  SDA_OUT;
  SOFT_IIC_delay(1);
  IIC_SDA_H;
  SOFT_IIC_delay(1);
  IIC_SCL_H;
  SOFT_IIC_delay(2);
  IIC_SCL_L;
}						     

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char SOFT_IIC_Wait_Ack(void)
@功能说明：模拟IIC等待应答信号
@参数说明：无
@函数返回：1，接收应答失败    0，接收应答成功
@修改时间：2022/02/24
@调用方法：SOFT_IIC_Wait_Ack();
@备    注：内部调用 有效应答：从机第9个 SCL=0 时 SDA 被从机拉低,并且 SCL = 1时 SDA依然为低
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char SOFT_IIC_Wait_Ack(void)
{
  unsigned char  ucErrTime=0;
  SDA_IN; //SDA设置为输入  （从机给一个低电平做为应答） 
  IIC_SCL_L;
  IIC_SDA_H;
  SOFT_IIC_delay(1);	   
  IIC_SCL_H;
  SOFT_IIC_delay(1);	 
  while(IIC_SDA_READ&0x1)
  {
    ucErrTime++;
    if(ucErrTime>200)
    {
      //SOFT_IIC_Stop();
      return 1;
    }
    SOFT_IIC_delay(1);	 
  }
  SOFT_IIC_delay(1);	 
  IIC_SCL_L; //时钟输出0 	   
  return 0;  
} 

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void IIC_Send_Byte(unsigned char data_t)
@功能说明：模拟IIC发送一个字节
@参数说明：data: 发送的字节
@函数返回：无
@修改时间：2022/02/24
@调用方法：IIC_SendByte(0x12);
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void SOFT_IIC_Send_Byte(unsigned char data_t)
{                        
  unsigned char  t;
      
  IIC_SCL_L; //拉低时钟开始数据传输
  SDA_OUT; 	
  for(t=0;t<8;t++)
  {   
    SOFT_IIC_delay(1);
    if(data_t&0x80)
    {
      IIC_SDA_H;
    }
    else 
    {
      IIC_SDA_L;
    }
    SOFT_IIC_delay(1);
    IIC_SCL_H;   
    data_t<<=1;
    SOFT_IIC_delay(2);   
    IIC_SCL_L;	     	
  }
  IIC_SDA_H;
  SOFT_IIC_delay(1);
} 	 

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char  SOFT_IIC_Read_Byte(unsigned char ack)
@功能说明：模拟IIC读取一个字节
@参数说明：ack=1 时，主机数据还没接收完 ack=0 时主机数据已全部接收完成
@函数返回：接收到的字节
@修改时间：2022/02/24
@调用方法：SOFT_IC_ReadByte(0x12);
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char SOFT_IIC_Read_Byte(unsigned char ack)
{
  unsigned char  i,tem=0;
  SDA_IN; //SDA设置为输入模式 等待接收从机返回数据
  IIC_SCL_L;
  IIC_SDA_L;
  for(i=0;i<8;i++ )
  {
    SOFT_IIC_delay(2);
    IIC_SCL_H; 
    if(IIC_SDA_READ) tem++; //从机发送的电平
    if(i < 7) tem <<= 1;
    SOFT_IIC_delay(2);    
    IIC_SCL_L;       
  }					 
  if(ack)
    SOFT_IIC_Ack(); //发送ACK 
  else
    SOFT_IIC_NAck(); //发送nACK  
  return tem;
}


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char IIC_ReadByteFromSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf)
@功能说明：模拟IIC读取指定设备 指定寄存器的一个值
@参数说明：I2C_Addr：目标设备地址 reg ：目标寄存器 buf ：存放读出字节
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：IIC_ReadByteFromSlave(0xD0, 0x75, buf);   //读 IIC地址为 0xD0器件（SOFT_I2C）寄存器0x75
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_ReadByteFromSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf)
{
  SOFT_IIC_Start();	
  SOFT_IIC_Send_Byte(I2C_Addr);	 //发送从机地址
  if(SOFT_IIC_Wait_Ack()) //如果从机未应答则数据发送失败
  {
    SOFT_IIC_Stop();
    return 1;
  }
  SOFT_IIC_Send_Byte(reg); //发送寄存器地址
  SOFT_IIC_Wait_Ack();	  
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(I2C_Addr+1); //进入接收模式			   
  SOFT_IIC_Wait_Ack();
  *buf=SOFT_IIC_Read_Byte(0);	   
  SOFT_IIC_Stop(); //产生一个停止条件
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char IIC_WriteByte(unsigned char I2C_Addr,unsigned char reg,unsigned char data_t)
@功能说明：模拟IIC写指定设备 指定寄存器的一个值
@参数说明：I2C_Addr：目标设备地址 reg：目标寄存器data：写入的数据
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：IIC_ReadByte(0xD0, 0X6B, 0X80);   //IIC地址为 0xD0器件（SOFT_I2C）寄存器0x6B 写入0x80
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_ReadByte(unsigned char I2C_Addr,unsigned char reg,unsigned char *buf)
{
  SOFT_IIC_Start();	
  SOFT_IIC_Send_Byte(I2C_Addr);	 //发送从机地址
  if(SOFT_IIC_Wait_Ack()) //如果从机未应答则数据发送失败
  {
    SOFT_IIC_Stop();
    return 1;
  }
  SOFT_IIC_Send_Byte(reg); //发送寄存器地址
  SOFT_IIC_Wait_Ack();	  
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(I2C_Addr+1); //进入接收模式			   
  SOFT_IIC_Wait_Ack();
  *buf=SOFT_IIC_Read_Byte(0);	   
  SOFT_IIC_Stop(); //产生一个停止条件
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char IIC_WriteByteToSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char data_t)
@功能说明：模拟IIC写指定设备 指定寄存器的一个值
@参数说明：I2C_Addr：目标设备地址 reg：目标寄存器data：写入的数据
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：IIC_ReadByteFromSlave(0xD0, 0X6B, 0X80);   //IIC地址为 0xD0器件（SOFT_I2C）寄存器0x6B 写入0x80
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_WriteByteToSlave(unsigned char I2C_Addr,unsigned char reg,unsigned char data_t)
{
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(I2C_Addr); //发送从机地址
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop();
    return 1; //从机地址写入失败
  }
  SOFT_IIC_Send_Byte(reg); //发送寄存器地址
  SOFT_IIC_Wait_Ack();	  
  SOFT_IIC_Send_Byte(data_t); 
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop(); 
    return 1; //数据写入失败
  }
  SOFT_IIC_Stop(); //产生一个停止条件
  
  //return 1; //status == 0;
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char IIC_ReadMultByteFromSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char* data_t)
@功能说明：模拟IIC写指定设备 指定寄存器的n个值
@参数说明：dev：目标设备地址  reg：目标寄存器  len：写入长度  data：存放写入数据
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：IIC_ReadMultByteFromSlave(0xD0, 0X6B, 1, 0X80);   //向寄存器0x6B写入0x80
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_ReadMultByteFromSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char *data_t)
{
  unsigned char  count = 0;
  unsigned char  temp;
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(dev); //发送从机地址
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop(); 
    return 1; //从机地址写入失败
  }
  SOFT_IIC_Send_Byte(reg); //发送寄存器地址
  SOFT_IIC_Wait_Ack();	  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(dev+1); //进入接收模式	
  SOFT_IIC_Wait_Ack();
  for(count=0;count<len;count++)
  {
    if(count!=(len-1))
      temp = SOFT_IIC_Read_Byte(1); //带ACK的读数据
    else  
      temp = SOFT_IIC_Read_Byte(0); //最后一个字节NACK
    
    data_t[count] = temp;
  }
  SOFT_IIC_Stop(); //产生一个停止条件
  //return count;
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char IIC_WriteMultByteToSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char* data_t)
@功能说明：模拟IIC写指定设备 指定寄存器的n个值
@参数说明：dev：目标设备地址  reg：目标寄存器  len：写入长度  data：存放写入数据
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：IIC_WriteMultByteToSlave(0xD0, 0X6B, 1, 0X80);   //向寄存器0x6B写入0x80
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char IIC_WriteMultByteToSlave(unsigned char dev, unsigned char reg, unsigned char len, unsigned char* data_t)
{
  
  unsigned char  count = 0;
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(dev); //发送从机地址
  if(SOFT_IIC_Wait_Ack())
  {
    SOFT_IIC_Stop();
    return 1; //从机地址写入失败
  }
  SOFT_IIC_Send_Byte(reg); //发送寄存器地址
  SOFT_IIC_Wait_Ack();	  
  for(count=0;count<len;count++)
  {
    SOFT_IIC_Send_Byte(data_t[count]); 
    if(SOFT_IIC_Wait_Ack()) //每一个字节都要等从机应答
    {
      SOFT_IIC_Stop();
      return 1; //数据写入失败
    }
  }
  SOFT_IIC_Stop(); //产生一个停止条件
  
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int LQ_I2C_Write(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf)
@功能说明：模拟SPI读写数据及长度
@参数说明：addr ：标设备地址  reg ：目标寄存器  len ：写入长度  *buf：存放写入数据
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：LQ_I2C_Write(0x68, 0x38, 1, tmp)
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LQ_I2C_Write(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *dat)
{
  unsigned int i = 0;
  
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(addr << 1);
  SOFT_IIC_Wait_Ack();
 
  SOFT_IIC_Send_Byte(reg);
  SOFT_IIC_Wait_Ack();
  
  for (i = 0; i < len; i++) 
  {
    SOFT_IIC_Send_Byte(dat[i]);
    SOFT_IIC_Wait_Ack();   
  }
  SOFT_IIC_Stop();
  return 0;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int LQ_I2C_Read(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf)
@功能说明：模拟SPI读写数据及长度
@参数说明：addr ：标设备地址  reg ：目标寄存器  len ：写入长度  *buf：存放写入数据
@函数返回：1失败 0成功
@修改时间：2022/02/24
@调用方法：LQ_I2C_Read(0x68, 0x38, 1, tmp)
@备    注：无
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int LQ_I2C_Read(unsigned char addr, unsigned char reg, unsigned int len, unsigned char *buf)
{
  SOFT_IIC_Start();
  SOFT_IIC_Send_Byte(addr << 1);
  SOFT_IIC_Wait_Ack();
  
  SOFT_IIC_Send_Byte(reg);
  SOFT_IIC_Wait_Ack();
  SOFT_IIC_delay(2);//应该需要一定延时。
  SOFT_IIC_Start();
  
  SOFT_IIC_Send_Byte((addr << 1)+1);
  SOFT_IIC_Wait_Ack();
  while (len) 
  {
    if (len == 1)
      *buf = SOFT_IIC_Read_Byte(0);
    else
      *buf = SOFT_IIC_Read_Byte(1);
    buf++;
    len--;
  }
  SOFT_IIC_Stop();
  return 0;
}
