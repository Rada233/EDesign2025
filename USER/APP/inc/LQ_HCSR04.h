#ifndef _HCSR04_DRIVER_H_
#define _HCSR04_DRIVER_H_

#include <stdio.h>
#include <stdint.h>

extern uint32_t TimeCounter;
extern uint32_t HalTime1, HalTime2;

void Ultrasonic_Init(void);
void LQ_TIM2_Init(uint16_t arr,uint16_t psc);
uint32_t Get_Distance(void);

void Test_Ultrasonic(void);

#endif

