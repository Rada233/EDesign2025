#ifndef __TIM_H
#define __TIM_H
#include "include.h"


extern TIM_HandleTypeDef TIM1_Handler;      //定时器句柄 
extern TIM_HandleTypeDef TIM2_Handler;      //定时器句柄 
extern TIM_HandleTypeDef TIM3_Handler;      //定时器句柄 
extern TIM_HandleTypeDef TIM4_Handler;      //定时器句柄 

void LQ_TIM3_Init(u16 arr,u16 psc);
 #endif
