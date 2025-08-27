
#include "include.h"


void sensor_init(void)
{
	

    GPIO_InitTypeDef GPIO_Initure;
    
    __HAL_RCC_GPIOA_CLK_ENABLE();                       //开启GPIOA时钟
	__HAL_RCC_GPIOB_CLK_ENABLE();                        //开启GPIOB时钟

    GPIO_Initure.Pin=GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;  	    //PA4 5 6 7 
    GPIO_Initure.Mode=GPIO_MODE_INPUT;      			//输入
    GPIO_Initure.Pull=GPIO_PULLDOWN;        			//下拉
    GPIO_Initure.Speed=GPIO_SPEED_FREQ_HIGH;			//高速
    HAL_GPIO_Init(GPIOA,&GPIO_Initure);    
	
}

	
	
	
	
uint8_t Read_sensor(sensor_num num)
{
    switch (num)
    {
    case 0:
        if (HAL_GPIO_ReadPin(sensor1_PIN) == GPIO_PIN_RESET)
            return 1;
        break;
    case 1:
        if (HAL_GPIO_ReadPin(sensor2_PIN) == GPIO_PIN_RESET)
            return 1;
        break;
    case 2:
        if (HAL_GPIO_ReadPin(sensor3_PIN) == GPIO_PIN_RESET)
            return 1;
        break;
    case 3:
        if (HAL_GPIO_ReadPin(sensor4_PIN) == GPIO_PIN_RESET)
            return 1;
        break;
        }
    return 0;
}

void sensor_Test(void)
{
	uint8_t sensor_Value[4];
	char txt[20];
	OLED_Init(); // OLED初始化
	LED_Init();
	delay_ms(200);
	OLED_CLS();
	sensor_init();
	
	while(1)
	{
		sensor_Value[0] = Read_sensor(sensor1);
		sensor_Value[1] = Read_sensor(sensor2);
		sensor_Value[2] = Read_sensor(sensor3);
		sensor_Value[3] = Read_sensor(sensor4);
		sprintf(txt, "%d %d %d %d", sensor_Value[0], sensor_Value[1], sensor_Value[2], sensor_Value[3]);
		OLED_P6x8Str(0, 2, txt); // 字符串
		 LED_Ctrl(RVS); 
		delay_ms(200);
	}
}
