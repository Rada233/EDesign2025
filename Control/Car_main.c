#include "Car_main.h"
#include "include.h"

#include "stdlib.h"

#define WAY_Flag 1		

Car Target_V; 
Car ENC_V;	  
Car Moto_PWM; 

uint8_t Motor_Flag = 0; 
float E_V = 0.0;
int32_t car_V;
sensor Car_sensor;
int i;

// ==== ?????? ====
uint8_t back_flag = 0;       // ??????
uint32_t back_counter = 0;   // ??????(??:????)

void Car_main(void)
{
	LED_Init();						
	KEY_Init();						
	OLED_Init();    				
	OLED_CLS();						
	uart_init(USART_2,115200);		
	uart_init(USART_3,115200);		
	Encoder_Init_TIM2();			
	Encoder_Init_TIM3();			
	Encoder_Init_TIM4();			
	MotorInit();					
	sensor_init();                  
	
	while(1)
	{
		Motor_Flag = 1;
		car_tim();
		OLED_Task();	
	}
}

void car_tim(void)
{
#if	WAY_Flag
    Car_sensor.a = Read_sensor(sensor2);
    Car_sensor.b = Read_sensor(sensor4);
    Car_sensor.c = Read_sensor(sensor3);
    Car_sensor.d = Read_sensor(sensor1);
#else
    Car_sensor.a = -Read_sensor(sensor2);
    Car_sensor.b = -Read_sensor(sensor4);
    Car_sensor.c = -Read_sensor(sensor3);
    Car_sensor.d = -Read_sensor(sensor1);
#endif
    E_V = (Car_sensor.a * 2 + Car_sensor.b * 1.2) - (Car_sensor.c * 1.2 + Car_sensor.d * 2);

    if (back_flag)   
    {
        car_V = -800;  
        if (back_counter > 0)
        {
            back_counter--;   
        }
        else
        {
            back_flag = 0;    
        }
    }
    else
    {
#if	WAY_Flag
        if (Car_sensor.a == 1 && Car_sensor.b == 1 && Car_sensor.c == 1 && Car_sensor.d == 1)
        {
            car_V = -800;   
            back_flag = 1;           
            back_counter = 800 / 100; 
        }
        else
        {
            if (abs(E_V) > 2) 
                car_V = 600;
            else 
                car_V = 800;
        }
#else
        if (Car_sensor.a == 0 && Car_sensor.b == 0 && Car_sensor.c == 0 && Car_sensor.d == 0)
        {
            car_V = -800;   
            back_flag = 1;
            back_counter = 500 / 100; 
        }
        else
        {
            if (abs(E_V) > 2) 
                car_V = 600;
            else 
                car_V = 800;
        }
#endif
    }
		
		Target_V.L = car_V + E_V * 280;
		Target_V.R = -car_V + E_V * 280;
		Target_V.B = E_V * 300;

    if (Motor_Flag)
    {
        Moto_PWM.L = Target_V.L;
        Moto_PWM.R = Target_V.R;
        Moto_PWM.B = Target_V.B;
    }
    else		
    {
        Moto_PWM.L = 0;
        Moto_PWM.R = 0;
        Moto_PWM.B = 0;
    }

    Moto_PWM.L = ((Moto_PWM.L) < (-6000) ? (-6000) : ((Moto_PWM.L) > (6000) ? (6000) : (Moto_PWM.L)));
    Moto_PWM.R = ((Moto_PWM.R) < (-6000) ? (-6000) : ((Moto_PWM.R) > (6000) ? (6000) : (Moto_PWM.R)));
    Moto_PWM.B = ((Moto_PWM.B) < (-6000) ? (-6000) : ((Moto_PWM.B) > (6000) ? (6000) : (Moto_PWM.B)));

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);

    ENC_V.L = Read_Encoder(4);  
    ENC_V.R = Read_Encoder(2);  
    ENC_V.B = Read_Encoder(3);
}



void OLED_Task(void)
{
	char txt[64];
	
	sprintf(txt, "%d %d %d %d E:%.1f", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d, E_V);
	OLED_P6x8Str(0, 2, txt);
	sprintf(txt, "ENC: %d %d %d ", ENC_V.L, ENC_V.R, ENC_V.B);
	OLED_P6x8Str(0, 3, txt);
	sprintf(txt, "Tar: %d %d %d", Target_V.L, Target_V.R, Target_V.B);
	OLED_P6x8Str(0, 4, txt);
	printf("samples:%d,%d,%d\n", ENC_V.L, ENC_V.R, ENC_V.B);
	delay_ms(100);   
}
