#include "zf_common_headfile.h"
#include "string.h"

#define PIT_CH                          (TIM1_PIT )          //中断优先级最低                   //使用的周期中断编号 

uint8 data_buffer[32];
uint8 data_len;
uint32 time_count = 0;

#define ENCODER_DIR_1                	(TIM3_ENCOEDER)                         //定义左、右编码器引脚
#define ENCODER_DIR_DIR_1           	(IO_P13)             					
#define ENCODER_DIR_PULSE_1       		(TIM3_ENCOEDER_P04)   

#define ENCODER_DIR_2                 	(TIM0_ENCOEDER)                         
#define ENCODER_DIR_DIR_2              	(IO_P35)            				 	
#define ENCODER_DIR_PULSE_2            	(TIM0_ENCOEDER_P34)            			         		

int16 encoder_L = 0;                                                   //存储编码器采集到的数据
int16 encoder_R = 0;

#define PWM_L1              (PWMB_CH1_P50)                                      //定义左、右电机引脚
#define PWM_L2              (PWMB_CH4_P53)

#define PWM_R1              (PWMB_CH2_P51)
#define PWM_R2              (PWMB_CH3_P52)

#define MAX_DUTY            (30)                //设置最大占空比50%
double L_MOTOR_Duty=18, R_MOTOR_Duty=18;				//设置左右电机占空比

typedef struct PID                              //PID结构体
{
	double iError;     //本次偏差
    double LastError;  //上次偏差
    double PrevError;  //上上次偏差
    double KP;
    double KI;
    double KD;
}PID;

PID Left_MOTOR_PID, Right_MOTOR_PID;                //左右轮PID
double L_MOTOR_PID[3] = {0.4, 0.002, 0.00005};                  //左右轮速度环PID参数
double R_MOTOR_PID[3] = {0.3, 0.001, 0.0001};
double Left_High_Speed=40, Right_High_Speed=40;           //左右电机目标编码器数值

void pit_handler (void);                                                        //周期中断处理函数
double PID_speed(PID*p, double ActualSpeed, double SetSpeed, double *MOTOR_PID);        //增量式PID（左右轮电机速度闭环控制）
double motor_limit(double duty);                          //电机限幅保护

void main()
{
    clock_init(SYSTEM_CLOCK_40M);
	debug_init();								

    if(wireless_uart_init())                                                    //判断是否通过初始化
    {
        while(1)                                                                //初始化失败进入死循环
        {
            
        }
    }

    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);   	//编码器初始化
    encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);    
		
	pwm_init(PWM_L1, 17000, (uint32)(L_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                                                 //初始化电机占空比为0
    pwm_init(PWM_L2, 17000, 0);                                                 
    pwm_init(PWM_R1, 17000, (uint32)(R_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                                                 
    pwm_init(PWM_R2, 17000, 0);                                                 
	
	tim1_irq_handler = pit_handler;                 //周期中断回调函数
	
    memset(&Left_MOTOR_PID, 0, sizeof(Left_MOTOR_PID));
    memset(&Right_MOTOR_PID, 0, sizeof(Right_MOTOR_PID));

    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
	wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");  

    pit_ms_init(PIT_CH, 10);                                                   //定时器周期初始化100ms

	while(1)
    {
		
    }
}

void pit_handler (void)
{
    time_count++;

    encoder_L = -encoder_get_count(ENCODER_DIR_1);                     //获取编码器计数(编码器脉冲数直接反映了电机轴的转动量)
    encoder_R = encoder_get_count(ENCODER_DIR_2);             

    L_MOTOR_Duty += PID_speed(&Left_MOTOR_PID, encoder_L, Left_High_Speed, L_MOTOR_PID);
	R_MOTOR_Duty += PID_speed(&Right_MOTOR_PID, encoder_R, Right_High_Speed, R_MOTOR_PID);

    L_MOTOR_Duty=motor_limit(L_MOTOR_Duty);
    R_MOTOR_Duty=motor_limit(R_MOTOR_Duty);

    pwm_set_duty(PWM_L1,(uint32)(L_MOTOR_Duty * (PWM_DUTY_MAX / 100)));         //设置电机占空比         
    pwm_set_duty(PWM_L2, 0);                                            

    pwm_set_duty(PWM_R1,(uint32)(R_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                    
    pwm_set_duty(PWM_R2, 0);                                                                              

    encoder_clear_count(ENCODER_DIR_1);                                		   //清空编码器计数
    encoder_clear_count(ENCODER_DIR_2); 
    
    if(time_count==50){
        sprintf((char *)data_buffer, "Lduty=%.2f Rduty=%.2f %d %d",L_MOTOR_Duty,R_MOTOR_Duty,encoder_L,encoder_R);
        data_len = strlen(data_buffer);
        wireless_uart_send_buffer(data_buffer, data_len);
        time_count=0;
    }
}

double PID_speed(PID*p, double ActualSpeed, double SetSpeed, double *MOTOR_PID)
{
	double Increase;    
	p->KP = *MOTOR_PID;    //参数赋值
	p->KI = *(MOTOR_PID+1);
	p->KD = *(MOTOR_PID+2);

	p->iError = SetSpeed - ActualSpeed;  

    p->iError = (p->iError>100)?100:((p->iError<-100)?-100:p->iError); // 积分项的限幅处理

	Increase = p->KP * (p->iError - p->LastError)
			 + p->KI * p->iError
			 + p->KD * (p->iError - 2*p->LastError + p->PrevError);

	p->PrevError = p->LastError;  
	p->LastError = p->iError;  

	return Increase;
}

double motor_limit(double duty){
    duty=(duty<0)?0:(duty>MAX_DUTY)?MAX_DUTY:duty;
    return duty;
}
