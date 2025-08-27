/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
【平    台】北京龙邱智能科技自平衡独轮车
【编    写】chiusir
【E-mail  】chiusir@163.com
【软件版本】V1.1 
【最后更新】2022年3月10日
【相关信息参考下列地址】
【网    站】http://www.lqist.cn
【淘宝店铺】http://longqiu.taobao.com
------------------------------------------------
【dev.env.】Keil5.2及以上版本
【Target 】 STM32F103
【Crystal】 8.000Mhz
【SYS PLL】 72MHz
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
#include "include.h"
#include "balance.h"

extern short   gyro[3], accel[3], sensors;
extern float   Pitch, Roll, Yaw; 

u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //蓝牙遥控相关的变量
u8 Flag_Stop=1,Flag_Show=0;                 //停止标志位和 显示标志位 默认停止 显示打开
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Motor1,Motor2;                            //电机PWM变量 应是Motor的 向Moto致敬	
int Temperature;                            //显示温度
int Voltage;                                //电池电压采样相关的变量
float Angle_Balance,Gyro_Balance,Gyro_Turn; //平衡倾角 平衡陀螺仪 转向陀螺仪
float Show_Data_Mb;                         //全局显示变量，用于显示需要查看的数据
u32 Distance;                               //超声波测距
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send; //延时和调参等变量
float Acceleration_Z;                       //Z轴加速度计  
//float Balance_Kp=97 ,Balance_Kd=0.45,Velocity_Kp=3,Velocity_Ki=0.016;//PID参数0
float Balance_Kp=105 ,Balance_Kd=0.45,Velocity_Kp=2.8,Velocity_Ki=0.015;//PID参数0
//u16 PID_Parameter[10],Flash_Parameter[10];  //Flash相关数组

float Balance_zero =-9.8;

int Left1=0,Left2=0,Right2=0,Right1=0;

int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
u8 Flag_Target;
int Voltage_Temp,Voltage_Count,Voltage_All;
int Start_Flag=1;
float Turn_flag = 0;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void balance_car(void)
@功能说明：平衡功能实现
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：MPU6050 INT引脚5毫秒中断	
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void balance_car(void)
{
	    LED_Ctrl(RVS); 
		  Flag_Target=!Flag_Target;
		  if(delay_flag==1)
			 {
				 if(++delay_50==10)	 delay_50=0,delay_flag=0;                     //给主函数提供50ms的精准延时
			 }				 
		 if(Flag_Target==1)                                                  //5ms读取一次陀螺仪和加速度计的值，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
			{
			  LQ_DMP_Read(); 
				Gyro_Turn=gyro[2];               //===更新转向角速度
				Acceleration_Z=accel[2];         //===更新Z轴加速度计                                              
			}                                                                   //10ms控制一次，为了保证M法测速的时间基准，首先读取编码器数据
    Encoder_Left = Read_Encoder(3);  //读取编码器的值 
		Encoder_Right = -Read_Encoder(4);  //读取编码器的值
    LQ_DMP_Read();                                             //===更新姿态
		if(Pitch>30)
		{
			Pitch=30;			
		}
		if(Pitch<-10)
		{
			Pitch=-10;			
		}				
		Gyro_Turn=gyro[2];               //===更新转向角速度
		Acceleration_Z=accel[2];         //===更新Z轴加速度计                                              
	 // Key(); 
		Infra_car();
		Balance_Pwm =Balance(-Pitch,-gyro[0]);                   //===平衡PID控制	
	  Velocity_Pwm=velocity(Encoder_Left,Encoder_Right);                  //===速度环PID控制		 								 
	  Turn_Pwm    =turn(Encoder_Left,Encoder_Right,Gyro_Turn);            //===转向环PID控制     
 	  Motor1=Balance_Pwm+Velocity_Pwm-Turn_Pwm;                            //===计算左轮电机最终PWM
 	  Motor2=Balance_Pwm+Velocity_Pwm+Turn_Pwm;                            //===计算右轮电机最终PWM
   	Xianfu_Pwm();    
			
		if(Motor1>0)		Motor1 += 350;
		else if(Motor1<0)		Motor1 -= 350;	
		if(Motor2>0)		Motor2 += 350;
		else if(Motor2<0)		Motor2 -= 350;	
		
		if(Start_Flag==1)
		{
			Motor1 =Motor1*1.1;
			MotorCtrl(Motor1,Motor2);
		}
		else
		{
			MotorCtrl(0,0);	
		}
}
	
/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int Balance(float Angle,float Gyro)
@功能说明：直立PD控制
@参数说明：角度、角速度
@函数返回：直立控制PWM
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int Balance(float Angle,float Gyro)
{  
   float Bias;
	 int balance;
	 Bias=Angle-Balance_zero;                       //===求出平衡的角度中值 和机械相关
	 balance=Balance_Kp*Bias+Gyro*Balance_Kd;   //===计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	 return balance;
}
float Movement =30;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int velocity(int encoder_left,int encoder_right)
@功能说明：速度PI控制 修改前进后退速度，请修Target_Velocity，比如，改成60就比较慢了
@参数说明：左轮编码器、右轮编码器
@函数返回：速度控制PWM
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int velocity(int encoder_left,int encoder_right)
{  
    static float Velocity,Encoder_Least,Encoder;
	  static float Encoder_Integral,Target_Velocixty;
//	  //=============遥控前进后退部分=======================// 
//	  if(Bi_zhang==1&&Flag_sudu==1)  Target_Velocity=55;                 //如果进入避障模式,自动进入低速模式
//    else 	                         Target_Velocity=110;                 
//		if(1==Flag_Qian)    	Movement=-Target_Velocity/Flag_sudu;	         //===前进标志位置1 
//		else if(1==Flag_Hou)	Movement=Target_Velocity/Flag_sudu;         //===后退标志位置1
//	  else  Movement=0;	
//	  if(Bi_zhang==1&&Distance<500&&Flag_Left!=1&&Flag_Right!=1)        //避障标志位置1且非遥控转弯的时候，进入避障模式
//	  Movement=Target_Velocity/Flag_sudu;
   //=============速度PI控制器=======================//	
		Encoder_Least =(encoder_left+encoder_right)-Movement;                    //===获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零） 
		Encoder *= 0.8;		                                                //===一阶低通滤波器       
		Encoder += Encoder_Least*0.2;	                                    //===一阶低通滤波器    
		//Encoder_Integral +=Encoder;                                       //===积分出位移 积分时间：10ms
		Encoder_Integral+=(Encoder-Movement);                   //===接收遥控器数据，控制前进后退
		if(Encoder_Integral>10000)  	Encoder_Integral =10000;             //===积分限幅
		if(Encoder_Integral<-10000)	Encoder_Integral=-10000;              //===积分限幅	
		Velocity=Encoder*Velocity_Kp+Encoder_Integral*Velocity_Ki;        //===速度控制	
		if(Start_Flag==0)   Encoder_Integral=0;      //===电机关闭后清除积分
	  return Velocity;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int turn(int encoder_left,int encoder_right,float gyro)//转向控制
@功能说明：转向控制  修改转向速度，请修改Turn_Amplitude即可
@参数说明：左轮编码器、右轮编码器、Z轴陀螺仪
@函数返回：转向控制PWM
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int turn(int encoder_left,int encoder_right,float gyro)//转向控制
{
	 static float Turn_Target,Turn,Encoder_temp,Turn_Convert=0.9,Turn_Count;
	  float Turn_Amplitude=88/Flag_sudu,Kp=25,Kd=0.3;     
//	  //=============遥控左右旋转部分=======================//
//  	if(1==Left_Flag||1==Right_Flag)                      //这一部分主要是根据旋转前的速度调整速度的起始速度，增加小车的适应性
//		{
//			if(++Turn_Count==1)
//			Encoder_temp=myabs(encoder_left+encoder_right);
//			Turn_Convert=50/Encoder_temp;
//			if(Turn_Convert<0.6)Turn_Convert=0.6;
//			if(Turn_Convert>3)Turn_Convert=3;
//		}	
//	  else
//		{
//			Turn_Convert=0;
//			Turn_Count=0;
//			Encoder_temp=0;
//		}			
//		
//		
//		if(1==Flag_Left)	           Turn_Target-=Turn_Convert;
//		else if(1==Flag_Right)	     Turn_Target+=Turn_Convert; 
//		else Turn_Target=0;
	
//    if(Turn_Target>Turn_Amplitude)  Turn_Target=Turn_Amplitude;    //===转向速度限幅
//	  if(Turn_Target<-Turn_Amplitude) Turn_Target=-Turn_Amplitude;
//		if(Flag_Qian==1||Flag_Hou==1)  Kd=0.5;        
//		else Kd=0;   //转向的时候取消陀螺仪的纠正 有点模糊PID的思想
  	//=============转向PD控制器=======================//
		//Turn=-Turn_Target*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
	  Turn=-Turn_flag*Kp-gyro*Kd;                 //===结合Z轴陀螺仪进行PD控制
		return Turn;
}


/**************************************************************************
按键指令
**************************************************************************/
void Key(void)
{	
static u8 flag=0;
//flag=click_N_Double(70);	
if(flag==1&&Start_Flag==0)Start_Flag=1,flag=0; //单击一次启动标志位反转
if(flag==1&&Start_Flag==1)Start_Flag=0,flag=0; //单击一次启动标志位反转
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Xianfu_Pwm(void)
@功能说明：限制PWM赋值
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Xianfu_Pwm(void)
{	
	  int Amplitude=6900;    //===PWM满幅是7200 限制在6900
//		if(Flag_Qian==1)  Moto1+=DIFFERENCE;  //DIFFERENCE是一个衡量平衡小车电机和机械安装差异的一个变量。直接作用于输出，让小车具有更好的一致性。
//	  if(Flag_Hou==1)   Moto2-=DIFFERENCE;
    if(Motor1<-Amplitude) Motor1=-Amplitude;	
		if(Motor1>Amplitude)  Motor1=Amplitude;	
	  if(Motor2<-Amplitude) Motor2=-Amplitude;	
		if(Motor2>Amplitude)  Motor2=Amplitude;		
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：int myabs(int a)
@功能说明：绝对值函数
@参数说明：int
@函数返回：unsigned int
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
int myabs(int a)
{ 		   
	  int temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void OLED_show(void)
@功能说明：OLED显示函数
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void OLED_show(void)
{
	char txt[30];
	sprintf(txt,"L1:%01d L2:%01d",Left1,Left2);
	OLED_P6x8Str(10 ,2,(u8*)txt);
	sprintf(txt,"R2:%01d R1:%01d",Right2,Right1);
	OLED_P6x8Str(10 ,3,(u8*)txt);
	sprintf(txt,"%05d %05d",Encoder_Left,Encoder_Right);
	OLED_P6x8Str(10 ,4,(u8*)txt);
	sprintf(txt,"%05d %05d",Motor1,Motor2);
	OLED_P6x8Str(10 ,5,(u8*)txt);
	sprintf(txt,"%.02f %06d",Pitch,gyro[0]);
	OLED_P6x8Str(10 ,6,(u8*)txt);

}

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Infra_Init(void)
@功能说明：红外循迹初始化
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Infra_Init(void)
{
	   GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();           //开启GPIOA时钟
		__HAL_RCC_GPIOB_CLK_ENABLE();           //开启GPIOB时钟

    GPIO_Initure.Pin=GPIO_PIN_2 | GPIO_PIN_3;  	//PA11  12
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);    
	
		GPIO_Initure.Pin=GPIO_PIN_0 | GPIO_PIN_1;  //PB0  1  2
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      								//输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        								//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;                //高速
    HAL_GPIO_Init(GPIOB,&GPIO_Initure); 
}
int left = 0,right = 0;

/*LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
@函数名称：void Infra_car(void)
@功能说明：红外循迹
@参数说明：无
@函数返回：无
@修改时间：2022/02/24
@备    注：
QQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQQ*/
void Infra_car(void)
{
	Left1  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_2);  // A2
	Left2  = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_3);	// A3
	Right2 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0); 	// B0
	Right1 = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1); 	// B1
	
	if(Left2==0&&Right2==0)
	{
		left = 0;
		right=0;	
	}
	else if(Left2==1&&Right2==0)
	{
		left = 1;
		right=0;		
	}
	else if(Left2==0&&Right2==1)
	{
		left = 0;
		right=1;
	}
	
	if(left==0&&right==0)
	{
		Turn_flag=0;
	//	Movement = 50;
	}
	else if(left==1&&right==0)
	{
		Turn_flag += 1;
	//	Movement = 50;
	}
		else if(left==0&&right==1)
	{
		Turn_flag -= 1;
	//	Movement= 50;
	}
	if(Turn_flag>20)Turn_flag=20;
	else if(Turn_flag<-20)Turn_flag=-20;
}