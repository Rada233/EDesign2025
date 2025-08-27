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
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/

#include "include.h"

#define MPU6050_ADDR  0x68  //IIC写入时的地址字节数据，+1为读取

unsigned char IIC_ICM20602 = 0;
unsigned char IIC_ICM20689 = 0;
unsigned char IIC_MPU9250 = 0;
unsigned char IIC_ICM42605 = 0;
unsigned char IIC_MPU6050 = 0;
unsigned char IIC_errorid = 0;

unsigned char Gyro_ID = 0;


/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char Gyro_Chose(void)
@功能说明：识别使用的陀螺仪
@参数说明：无
@函数返回：陀螺仪ID
@修改时间：2022/03/24
@调用方法：Gyro_Chose();
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char Gyro_Chose(void)
{
    Gyro_ID = MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);
    switch(Gyro_ID)
    {
        case 0x12:IIC_ICM20602 = 1;break;
        case 0x71:IIC_MPU9250 = 1;break;
        case 0x98:IIC_ICM20689 = 1;break;
        case 0x42:IIC_ICM42605 = 1;break;
        case 0x68:IIC_MPU6050 = 1;break;
        default :IIC_errorid = 1;return 0;
    }
    return Gyro_ID;
}


signed short  aacx,aacy,aacz;            //加速度传感器原始数据
signed short  gyrox,gyroy,gyroz;         //陀螺仪原始数据
short magx, magy, magz;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Test_IIC_Gyro(void)
@功能说明：读取MPU6050或者ICM20602原始数据 测试
@参数说明：void
@函数返回：
@修改时间：2022/03/15
@调用方法： Test_MPU6050();
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Test_IIC_Gyro(void)
{
	unsigned char res;
	char  txt[30];


	SOFT_IIC_Init();                  //IIC初始化//IIC初始化
	delay_ms(100);
	res = Gyro_Chose();                //判断陀螺仪型号
	sprintf(txt,"ID:0X%2x",res);

	OLED_Init();                //LCD初始化
	delay_ms(100);
	OLED_CLS();                 //LCD清屏
	OLED_P6x8Str(70,2,txt);     // 字符串显示
	if(IIC_ICM20602)
			OLED_P8x16Str(0,0,"LQ 20602 Test");// 字符串显示
	if(IIC_MPU6050)
			OLED_P8x16Str(0,0,"LQ 6050 Test");// 字符串显示
	if(IIC_ICM42605)
			OLED_P8x16Str(0,0,"LQ 42605 Test");// 字符串显示
	if(IIC_MPU9250)
			OLED_P8x16Str(0,0,"LQ 9250 Test");// 字符串显示
	if(IIC_ICM20689)
			OLED_P8x16Str(0,0,"LQ 20689 Test");// 字符串显示
	if(IIC_errorid)
	{
			OLED_P8x16Str(0,0,"Gyro Test fail");// 字符串显示
//			while(1);
	}

	MPU6050_Init();
	while(1)
    {
	    MPU_Get_Raw_data(&aacx,&aacy,&aacz,&gyrox,&gyroy,&gyroz);   //得到加速度传感器数据
	    if(IIC_MPU9250)
			{
				MPU9250_Get_Magnetometer(&magx, &magy, &magz);
				sprintf((char*)txt,"gx:%06d",magx);
				OLED_P6x8Str(70,5,txt);
				sprintf((char*)txt,"gy:%06d",magy);
				OLED_P6x8Str(70,6,txt);
				sprintf((char*)txt,"gz:%06d",magz);
				OLED_P6x8Str(70,7,txt);
			}
			sprintf((char*)txt,"ax:%06d",aacx);
			OLED_P6x8Str(0,2,txt);
			sprintf((char*)txt,"ay:%06d",aacy);
			OLED_P6x8Str(0,3,txt);
			sprintf((char*)txt,"az:%06d",aacz);
			OLED_P6x8Str(0,4,txt);
			sprintf((char*)txt,"gx:%06d",gyrox);
			OLED_P6x8Str(0,5,txt);
			sprintf((char*)txt,"gy:%06d",gyroy);
			OLED_P6x8Str(0,6,txt);
			sprintf((char*)txt,"gz:%06d",gyroz);
			OLED_P6x8Str(0,7,txt);
		//ANO_DT_send_int16byte16(aacx,aacy,aacz,gyrox,gyroy,gyroz,gyroz,gyroz);
	    delay_ms(100);
    }

}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void delayms_mpu(uint16 ms)
@功能说明：不精确延时
@参数说明：需要延时时间
@函数返回：无
@修改时间：2022/03/15
@调用方法：delayms_mpu(100);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void delayms_mpu(u16 ms)
{
	while(ms--)
	{
		u16  i = 300*50;
		while(i--)
		{
			__NOP();
		}
	}
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU6050_Init(void)
@功能说明：初始化MPU6050
@参数说明：无
@函数返回：0：初始化成功   1：失败
@修改时间：2022/03/15
@调用方法：MPU6050_Init();
@备    注：初始化调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU6050_Init(void)
{
    int  res;
    if (IIC_ICM42605)
    {
//        MPU_Write_Byte(MPU6050_ADDR, device_config_reg,bit_soft_reset_chip_config);//chip soft reset
//        delayms(10);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel,0x01);//Change to bank 1
//				MPU_Write_Byte(MPU6050_ADDR,intf_config4,0x03);//4 wire spi mode  四线SPI  默认值0x03
//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel,0x00);
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config_reg,0x40);//Stream-to-FIFO Mode  陀螺仪Y轴陷波滤波器的频率选择

//				res = MPU_Read_Byte(MPU6050_ADDR,int_source0_reg);
//				MPU_Write_Byte(MPU6050_ADDR,int_source0_reg,0x00);
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config2_reg,0x00);// watermark
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config3_reg,0x02);// watermark
//				MPU_Write_Byte(MPU6050_ADDR,int_source0_reg, (unsigned char)res);
//				MPU_Write_Byte(MPU6050_ADDR,fifo_config1_reg,0x63);// Enable the accel and gyro to the FIFO

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel,0x00);
//				MPU_Write_Byte(MPU6050_ADDR,int_config_reg,0x36);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = (MPU_Read_Byte(MPU6050_ADDR,int_source0_reg)|bit_int_fifo_ths_int1_en);
//				MPU_Write_Byte(MPU6050_ADDR,int_source0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,accel_config0_reg)&0x1F)|(bit_accel_ui_fs_sel_8g));//8g
//				MPU_Write_Byte(MPU6050_ADDR,accel_config0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,accel_config0_reg)&0xF0)|bit_accel_odr_50hz);
//				MPU_Write_Byte(MPU6050_ADDR,accel_config0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,gyro_config0_reg)&0x1F)|(bit_gyro_ui_fs_sel_1000dps));
//				MPU_Write_Byte(MPU6050_ADDR,gyro_config0_reg,(unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = ((MPU_Read_Byte(MPU6050_ADDR,gyro_config0_reg)&0xF0)|bit_gyro_odr_50hz);
//				MPU_Write_Byte(MPU6050_ADDR,gyro_config0_reg, (unsigned char)res);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = MPU_Read_Byte(MPU6050_ADDR,pwr_mgmt0_reg)|(bit_accel_mode_ln); // Accel on in LNM
//				MPU_Write_Byte(MPU6050_ADDR,pwr_mgmt0_reg, (unsigned char)res);
//				delayms(1);

//				MPU_Write_Byte(MPU6050_ADDR,reg_bank_sel, 0x00);
//				res = MPU_Read_Byte(MPU6050_ADDR,pwr_mgmt0_reg)|(bit_gyro_mode_ln); // Gyro on in LNM
//				MPU_Write_Byte(MPU6050_ADDR,pwr_mgmt0_reg,(unsigned char)res);
//				delay_ms(1);
				return 0;
    }
    else
    {
        res = MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);           //读取MPU6050的ID
        if(res != Gyro_ID)                                 //器件ID正确
            return 1;

        res = 0;
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU6050
        delayms_mpu(100);  //延时100ms
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU6050
        res += MPU_Set_Gyro_Fsr(3);					        	   //陀螺仪传感器,±2000dps
        res += MPU_Set_Accel_Fsr(1);					       	   //加速度传感器,±4g
        res += MPU_Set_Rate(1000);						       	   //设置采样率1000Hz
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,0x02);      //设置数字低通滤波器   98hz
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT1_REG,0X01);//设置CLKSEL,PLL X轴为参考
        res += MPU_Write_Byte(MPU6050_ADDR,MPU_PWR_MGMT2_REG,0X00);//加速度与陀螺仪都工作

        if(IIC_MPU9250)
        {
            MPU_Write_Byte(MPU6050_ADDR,MPU_FIFO_EN_REG,0X00);  //关闭FIFO
            MPU_Write_Byte(MPU6050_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式
            res = MPU_Read_Byte(AK8963_ADDR,MPU_MAG_WIA);   //读取MPU9250的ID
            if(res == AK8963_ID)                                //器件ID正确
            {
                MPU_Write_Byte(AK8963_ADDR,MPU_MAG_CNTL2_REG,0X01);//软件重置
                delayms_mpu(100);
                MPU_Write_Byte(AK8963_ADDR,MPU_MAG_CNTL1_REG,0X16);//设置为单次模式，输出16bit
            }
        }
        return 0;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Set_Gyro_Fsr(unsigned char fsr)
@功能说明：设置陀螺仪测量范围
@参数说明：fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
@函数返回：0：设置成功
@修改时间：2022/03/15
@调用方法：MPU9250_Set_Gyro_Fsr(3);  //陀螺仪传感器,±2000dps
@备    注：内部调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_Gyro_Fsr(unsigned char fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_GYRO_CFG_REG,fsr<<3);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Set_Accel_Fsr(uint8_t fsr)
@功能说明：设置加速度计测量范围
@参数说明：fsr:0,±2g;1,±4g;2,±8g;3,±16g
@函数返回：0：设置成功
@修改时间：2022/03/15
@调用方法：MPU_Set_Accel_Fsr(1); //加速度传感器,±4g
@备    注：内部调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_Accel_Fsr(unsigned char fsr)
{
	return MPU_Write_Byte(MPU6050_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Set_LPF(uint8_t fsr)
@功能说明：设置数字低通滤波
@参数说明：lpf:数字低通滤波频率(Hz)
@函数返回：0：设置成功
@修改时间：2022/03/15
@调用方法：MPU6050_Set_LPF(100);
@备    注：内部调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_LPF(u16 lpf)
{
	unsigned char  dat=0;
	if(lpf>=188)dat=1;
	else if(lpf>=98)dat=2;
	else if(lpf>=42)dat=3;
	else if(lpf>=20)dat=4;
	else if(lpf>=10)dat=5;
	else dat=6;
	return MPU_Write_Byte(MPU6050_ADDR,MPU_CFG_REG,dat);//设置数字低通滤波器
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Set_Rate(uint16_t rate)
@功能说明：设置采样率
@参数说明：rate:4~1000(Hz)
@函数返回：0：设置成功
@修改时间：2022/03/15
@调用方法：MPU_Set_Rate(1000); //设置采样率1000Hz
@备    注：内部调用
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Set_Rate(u16 rate)
{
	unsigned char  dat;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	dat=1000/rate-1;
	MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,dat);	    //设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	                                //自动设置LPF为采样率的一半
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：short MPU_Get_Temperature(void)
@功能说明：获取温度值
@参数说明：无
@函数返回：温度值(扩大了100倍)
@修改时间：2022/03/15
@调用方法：signed short temp = MPU_Get_Temperature();
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
short MPU_Get_Temperature(void)
{
    unsigned char  buf[2];
    short raw;
	float temp;
	MPU_Read_Len(MPU6050_ADDR,MPU_TEMP_OUTH_REG,2,buf);
    raw=((u16)buf[0]<<8)|buf[1];
    temp=21+((double)raw)/333.87;
    return (short)temp*100;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Get_Gyroscope(signed short *gx,signed short *gy,signed short *gz)
@功能说明：获取陀螺仪值
@参数说明：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
@函数返回：0：读取成功
@修改时间：2022/03/15
@调用方法：MPU_Get_Gyroscope(&data[0], &data[1], &data[2]);
@备    注：需定义 signed short data[6];
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Get_Gyroscope(signed short *gx,signed short *gy,signed short *gz)
{
    unsigned char  buf[6],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];
		*gy=((u16)buf[2]<<8)|buf[3];
		*gz=((u16)buf[4]<<8)|buf[5];
	}
    return res;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Get_Accelerometer(signed short *ax,signed short *ay,signed short *az)
@功能说明：获取加速度值
@参数说明：ax,ay,az:陀螺仪x,y,z轴的原始读数(带符号)
@函数返回：0：读取成功
@修改时间：2022/03/15
@调用方法：MPU_Get_Accelerometer(&data[0], &data[1], &data[2]);
@备    注：需定义 signed short data[6];
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Get_Accelerometer(signed short *ax,signed short *ay,signed short *az)
{
    unsigned char  buf[6],res;
	res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];
		*ay=((u16)buf[2]<<8)|buf[3];
		*az=((u16)buf[4]<<8)|buf[5];
	}
    return res;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Get_Raw_data(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz)
@功能说明：获取 加速度值 角速度值
@参数说明：gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)，ax,ay,az:陀螺仪x,y,z轴的原始读数(带符号)
@函数返回：0：读取成功
@修改时间：2022/03/15
@调用方法：MPU_Get_Raw_data(&data[0], &data[1], &data[2],&data[3], &data[4], &data[5]);
@备    注：需定义 signed short data[6];
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Get_Raw_data(signed short *ax,signed short *ay,signed short *az,signed short *gx,signed short *gy,signed short *gz)
{
    if(IIC_ICM42605)
    {
        unsigned char  buf[14],res;
        res=MPU_Read_Len(MPU6050_ADDR,0X1F,12,buf);
        if(res==0)
        {
            *ax=((u16)buf[0]<<8)|buf[1];
            *ay=((u16)buf[2]<<8)|buf[3];
            *az=((u16)buf[4]<<8)|buf[5];
            *gx=((u16)buf[6]<<8)|buf[7];
            *gy=((u16)buf[8]<<8)|buf[9];
            *gz=((u16)buf[10]<<8)|buf[11];
        }
        return res;
    }
    else
    {
        unsigned char  buf[14],res;

        res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
        if(res==0)
        {
            *ax=((u16)buf[0]<<8)|buf[1];
            *ay=((u16)buf[2]<<8)|buf[3];
            *az=((u16)buf[4]<<8)|buf[5];
            *gx=((u16)buf[8]<<8)|buf[9];
            *gy=((u16)buf[10]<<8)|buf[11];
            *gz=((u16)buf[12]<<8)|buf[13];
        }
        return res;
    }
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU9250_Get_Magnetometer(short *mx,short *my,short *mz)
@功能说明：获取磁力计值
@参数说明：mx,my,mz:磁力计仪x,y,z轴的原始读数(带符号)
@函数返回：0：读取成功
@修改时间：2022/03/15
@调用方法：MPU9250_Get_Magnetometer(&data[0], &data[1], &data[2]);
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU9250_Get_Magnetometer(short *mx,short *my,short *mz)
{

  unsigned char  buf[6],res;
  res=MPU_Read_Len(AK8963_ADDR,MPU_MAG_XOUTL_REG,6,buf);
  if(res==0)
  {
    *mx=((u16)buf[1]<<8)|buf[0];
    *my=((u16)buf[3]<<8)|buf[2];
    *mz=((u16)buf[5]<<8)|buf[4];
  }
  MPU_Read_Byte(AK8963_ADDR,MPU_MAG_ST2_REG);
  return res;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
@功能说明：IIC 连续读
@参数说明：addr:器件地址，reg :要读取的寄存器地址
@参数说明：len :要读取的长度，buf :读取到的数据存储区
@函数返回： 0 ：读取成功
@修改时间：2022/03/15
@调用方法：MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
@备    注：底层驱动 移植时需要修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Read_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_ReadMultByteFromSlave(addr<<1, reg, len, buf);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,
@                                         unsigned char len,unsigned char *buf)
@功能说明：IIC 连续写
@参数说明：addr:器件地址，reg :要写入的寄存器地址，len :要写入的长度
@参数说明：buf :写入到的数据存储区
@函数返回：0 ：写入成功
@修改时间：2022/03/15
@调用方法：MPU_Write_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,14,buf);
@备    注：底层驱动 移植时需要修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Write_Len(unsigned char addr,unsigned char reg,unsigned char len,unsigned char *buf)
{
    return IIC_WriteMultByteToSlave(addr<<1, reg, len, buf);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Write_Byte(unsigned char addr,
@                                           unsigned char reg,unsigned char value)
@功能说明：IIC 写一个寄存器
@参数说明：addr  :器件地址，reg   :寄存器地址，value :要写入的值
@函数返回：0 ：读取成功
@修改时间：2022/03/15
@调用方法：MPU_Write_Byte(MPU6050_ADDR,MPU_SAMPLE_RATE_REG,1);
@备    注：底层驱动 移植时需要修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Write_Byte(unsigned char addr,unsigned char reg,unsigned char value)
{
    return IIC_WriteByteToSlave(addr<<1, reg, value);
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：unsigned char MPU_Read_Byte(unsigned char addr,unsigned char reg)
@功能说明：IIC 读一个寄存器
@参数说明：addr  :器件地址，reg   :寄存器地址
@函数返回：读取的值
@修改时间：2022/03/15
@调用方法：MPU_Read_Byte(MPU6050_ADDR,WHO_AM_I);
@备    注：底层驱动 移植时需要修改
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
unsigned char MPU_Read_Byte(unsigned char addr,unsigned char reg)
{
    unsigned char value[1];
    MPU_Read_Len(addr, reg, 1, value);
    return value[0];
}
