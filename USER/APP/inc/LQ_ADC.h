
#ifndef __LQ_ADC_H
#define __LQ_ADC_H

#include "include.h"


void ADC_Init(void); 				//ADC通道初始化
u16  Get_Adc(u32 ch); 		        //获得某个通道值 
u16 Get_Adc_Average(u32 ch,u8 times);//得到某个通道给定次数采样的平均值

void Test_ADC(void);
int Get_Temp(uint32_t val);

#endif 







