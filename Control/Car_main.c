#include "Car_main.h"
#include "include.h"
#include "LQ_PID.h"

#define OBSTACLE_DIST 10    // 障碍检测阈值(cm)
#define TARGET_DIST 10      // 目标保持距离(cm)
#define CIRCLE_ANGLE 1600   // 环绕180°编码器阈值
#define ROTATE_ANGLE 1600   // 自转180°编码器阈值
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

#define WAY_Flag 1		

// 全局变量
Car Target_V, ENC_V, Moto_PWM;
pid_param_t PID_L, PID_R, PID_DIST;
uint16_t Dis = 0;
uint32_t encode_counter = 0;
uint8_t back_flag = 0;
uint32_t back_counter = 0;
sensor Car_sensor; 
float E_V;
int32_t base_speed;

// 状态机定义
typedef enum {
    run,        // 正常循迹
    circling,   // 环绕障碍
    rotating    // 自转恢复
} State_car;

State_car Car_State = run;

void Motor_Update(void)
{

    Moto_PWM.L = Target_V.L;
    Moto_PWM.R = Target_V.R;
    Moto_PWM.B = Target_V.B;

    // 输出限幅
    Moto_PWM.L = LIMIT(Moto_PWM.L, -6000, 6000);
    Moto_PWM.R = LIMIT(Moto_PWM.R, -6000, 6000);
    Moto_PWM.B = LIMIT(Moto_PWM.B, -6000, 6000);

    MotorCtrl3w(Moto_PWM.R, Moto_PWM.B, Moto_PWM.L);

    // 更新编码器
    ENC_V.L = Read_Encoder(4);  
    ENC_V.R = Read_Encoder(2);  
    ENC_V.B = Read_Encoder(3);
}


void Car_main(void)
{
    // 外设初始化
    LED_Init();                         
    MotorInit();                    
    sensor_init();
    Ultrasonic_Init();
    
    // PID初始化
    PidInit(&PID_L);    
    PidInit(&PID_R);
    PidInit(&PID_DIST);
    PID_DIST.kp = 0.04;  // 距离控制参数
    PID_DIST.ki = 0.0;
    PID_DIST.kd = 0.0;

    while(1)
    {
        Dis = Get_Distance();  // 获取实时距离
        
        // 状态切换检测
        if(Car_State == run && Dis <= OBSTACLE_DIST){
            Car_State = circling;
            encode_counter = 0;
        }
        OLED_Task();
        Control();       // 核心控制
        Motor_Update();  // 电机更新
    }
}


void car_tim(void)
{
	
	
    Car_sensor.a = Read_sensor(sensor2);
    Car_sensor.b = Read_sensor(sensor4);
    Car_sensor.c = Read_sensor(sensor3);
    Car_sensor.d = Read_sensor(sensor1);
        
    	
		E_V=(Car_sensor.a*2 + Car_sensor.b*1.2) - (Car_sensor.c*1.2 + Car_sensor.d*2);
    
    
    // ==== 添加倒车逻辑 ====
    if (back_flag) {
        base_speed = -800;
        if(back_counter > 0) back_counter--;
        else back_flag = 0;
    } 
    else {
#if WAY_Flag
        if (Car_sensor.a && Car_sensor.b && Car_sensor.c && Car_sensor.d) {
            base_speed = -800;
            back_flag = 1;
            back_counter = 500 / 100;
        } else {
            base_speed = (abs(E_V) > 2) ? 600 : 800;
        }
#else
        if (!Car_sensor.a && !Car_sensor.b && !Car_sensor.c && !Car_sensor.d) {
            base_speed = -800;
            back_flag = 1;
            back_counter = 500 / 100; 
        } else {
            base_speed = (abs(E_V) > 2) ? 600 : 800;
        }
#endif
    }
    // ==== 倒车逻辑结束 ====
    
    Target_V.L = base_speed + E_V * 280;
    Target_V.R = -base_speed + E_V * 280;
    Target_V.B = E_V * 300;
}

void OLED_Task(void)
{
    char txt[64];
    
    // 传感器数据显示
    sprintf(txt, "%d %d %d %d E:%.1f", Car_sensor.a, Car_sensor.b, Car_sensor.c, Car_sensor.d, E_V);
    OLED_P6x8Str(0, 2, txt);
    
    // 编码器数据显示
    sprintf(txt, "ENC: %d %d %d", ENC_V.L, ENC_V.R, ENC_V.B);
    OLED_P6x8Str(0, 3, txt);
    
    // 目标速度显示
    sprintf(txt, "Tar: %d %d %d", Target_V.L, Target_V.R, Target_V.B);
    OLED_P6x8Str(0, 4, txt);
    
    // 状态显示（枚举类型转换为整数）
    sprintf(txt, "State:%u", (unsigned int)Car_State);
    OLED_P6x8Str(0, 5, txt);
    
    // 距离显示（uint16_t类型）
    sprintf(txt, "Dist:%ucm", (unsigned int)Dis);
    OLED_P6x8Str(0, 6, txt);
    
    delay_ms(100);
}


void Control(void)
{
    static int circle_dir = 1;   // +1 左绕，-1 右绕（可以根据障碍物位置决定）

    switch (Car_State)
    {
    case run:  // 正常循迹
        car_tim();  
        break;

    case circling:  // 固定半径环绕模式
    {
        float radius = TARGET_DIST / 100.0f;  // cm -> m
        float v = 0.1f;                        // 环绕线速度 (m/s), 可调
        float omega = v / radius;              // 角速度 (rad/s), 保证车头始终对圆心

        // === 车体速度分解，保证车头指向圆心 ===
        float Vx = 0.0f;
        float Vy = circle_dir * v;
        float W  = circle_dir * omega;

        // === 三轮速度换算（考虑车体半径 R 和轮子半径 r_wheel） ===
        float R = 0.095f;       // 小车半径 (m)
        float r_wheel = 0.028f; // 轮子半径 (m)

        Target_V.L = (-0.5f * Vx +  0.866f * Vy + W * R) / r_wheel;
        Target_V.R = (-0.5f * Vx + -0.866f * Vy + W * R) / r_wheel;
        Target_V.B = ( 1.0f * Vx +  0.0f   * Vy + W * R) / r_wheel;

        // === 判断是否转够180° ===
        // 注意：这里仍使用B轮编码器累积计数，如果有陀螺仪更准确
        if((encode_counter += abs(ENC_V.B)) > CIRCLE_ANGLE){
            Car_State = rotating;
            encode_counter = 0;
        }
    }
    break;

    case rotating:  // 自转180°恢复原向
    {
        float R = 0.095f;       // 小车半径 (m)
        float r_wheel = 0.028f; // 轮子半径 (m)
        float rotate_speed = 1.0f; // rad/s, 自转速度，可调

        // 自转 180°，绕中心原地旋转
        Target_V.L = -rotate_speed * R / r_wheel;
        Target_V.R =  rotate_speed * R / r_wheel;
        Target_V.B = 0;

        if((encode_counter += abs(ENC_V.B)) > ROTATE_ANGLE){
            Car_State = run;  // 回到循迹
            encode_counter = 0;
        }
    }
    break;
    }
}