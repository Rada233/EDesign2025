#ifndef _CAR_MAIN_H
#define _CAR_MAIN_H
#include "stdint.h"

typedef struct
{
    int32_t L;
    int32_t R;
    int32_t B;
} Car;

typedef struct
{
    int8_t a;
    int8_t b;
    int8_t c;
    int8_t d;
} sensor;


typedef struct {
	char flag;     //标志位
	int  L_V;  //左轮速度
	int  R_V;  //右轮速度
	int  B_V;  //后轮速度
	
} Color_V;

void Car_main(void);
void car_tim(void);
void SYS_Task(void);
void OLED_Task(void);
void Control(void);
#endif
