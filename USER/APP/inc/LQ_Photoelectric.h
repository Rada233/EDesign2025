#ifndef _PHOTOELECTRIC_H_
#define _PHOTOELECTRIC_H_

#include "stdint.h"

#define sensor1_PIN  GPIOA,GPIO_PIN_4
#define sensor2_PIN  GPIOA,GPIO_PIN_7
#define sensor3_PIN  GPIOA,GPIO_PIN_6
#define sensor4_PIN  GPIOA,GPIO_PIN_5




// π‚µÁ—°‘Ò
typedef enum
{
	
	sensor1 = 0,
	sensor2,
	sensor3,
	sensor4,
	
} sensor_num;

void sensor_init(void);
uint8_t Read_sensor(sensor_num num);

void sensor_Test(void);

#endif
