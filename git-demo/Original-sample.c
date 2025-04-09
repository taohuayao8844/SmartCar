#include "zf_common_headfile.h"
#include "math.h"
#include "string.h"

#define PIT_CH                          (TIM1_PIT )             //使用的周期中断编号 
                                                                //TIM1中断优先级最低 
#define ADC_CHANNEL1            (ADC_CH14_P06)                  //定义使用的ADC通道                          
#define ADC_CHANNEL2            (ADC_CH13_P05)
#define ADC_CHANNEL3            (ADC_CH9_P01)
#define ADC_CHANNEL4            (ADC_CH0_P10)

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

int L1 = 0, L2 = 0, R2 = 0, R1 = 0;				    //四路电感值
double ADC_bias = 0;                    //差比和偏差值
double Left_High_Speed = 0,Right_High_Speed = 0,Speed_bias = 0;  //左右电机目标速度,速度偏差调整
double L_MOTOR_Duty = 18,R_MOTOR_Duty = 18;            //左右电机占空比
double basic_Speed=30,straight_speed=40;           //基础速度，直线速度
int outtrack_flag=0;                //冲出赛道标志
int straight_count=0;               //直线计数
uint16 adc_max[4]={950,940,950,950};             //存储各电感最大值

typedef struct PID                              //PID结构体
{
	double iError;     //本次偏差
    double LastError;  //上次偏差
    double PrevError;  //上上次偏差
    double KP;
    double KI;
    double KD;
}PID;

PID Left_MOTOR_PID, Right_MOTOR_PID, Turn_PID;                    //左右轮PID，转向PID
double L_MOTOR_PID[3] = {0.4, 0.002, 0.00005};                  //PID参数设置
double R_MOTOR_PID[3] = {0.3, 0.001, 0.0001};
double Turn_pd[2]={0.6,0.2};          
                                                       
double PlacePID_Control(PID*p, double Now_bias, double Set_bias, double *Turn_pd);      ////位置式PD，计算给电机的偏差
double SpeedPID_Control(PID*p, double ActualSpeed, double SetSpeed, double *MOTOR_PID);        //增量式PID（左右轮电机速度闭环控制）
double motor_limit(double duty);                          //电机限幅保护

uint8 data_buffer[32];                                    //无线串口通信
uint8 data_len = 0;
uint32 time_count = 0;

void pit_handler (void);                                  //周期中断处理函数

void main()
{
    clock_init(SYSTEM_CLOCK_40M);
	debug_init();	
    
    if(wireless_uart_init())                                                    //判断是否成功初始化
    {
        while(1)                                                                //失败就进入死循环
        {
            
        }
    }
    adc_init(ADC_CHANNEL1, ADC_10BIT);                                          //adc通道初始化                             
    adc_init(ADC_CHANNEL2, ADC_10BIT);                                          
    adc_init(ADC_CHANNEL3, ADC_10BIT);                                         
    adc_init(ADC_CHANNEL4, ADC_10BIT);

    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);   	//编码器初始化
    encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);    
		
	pwm_init(PWM_L1, 17000,(uint32)(L_MOTOR_Duty * (PWM_DUTY_MAX / 100)));     //初始化电机占空比                                         //?????????0
    pwm_init(PWM_L2, 17000, 0);                                                 
    pwm_init(PWM_R1, 17000,(uint32)(R_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                                              
    pwm_init(PWM_R2, 17000, 0);                                                
	
	tim1_irq_handler = pit_handler;                                             //周期中断回调函数
	
    wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              //无线串口输出测试信息
    
    memset(&Turn_PID,0,sizeof(Turn_PID));                                       //变量初始化
    memset(&Left_MOTOR_PID,0,sizeof(Left_MOTOR_PID));        
    memset(&Right_MOTOR_PID,0,sizeof(Right_MOTOR_PID));        

    pit_ms_init(PIT_CH, 10);                                                   //定时器周期初始化100ms

	while(1)
    {
		
    }
}

void pit_handler (void)
{
    time_count++;
    L1=adc_mean_filter_convert(ADC_CHANNEL1,5);      //5次均值滤波转换结果
    L2=adc_mean_filter_convert(ADC_CHANNEL2,5);      
    R2=adc_mean_filter_convert(ADC_CHANNEL3,5);    
    R1=adc_mean_filter_convert(ADC_CHANNEL4,5);

    L1=(unsigned long)L1*100/adc_max[0];           //归一化  (0-100)
    L2=(unsigned long)L2*100/adc_max[1];
    R2=(unsigned long)R2*100/adc_max[2];
    R1=(unsigned long)R1*100/adc_max[3];

    if(L1<10&&L2<10&&R2<10&&R1<10){          //冲出赛道保护
        outtrack_flag=1;
    }
    else{
        outtrack_flag=0;
    }

    ADC_bias=(double)((L1 - R1)+(L2 - R2))*100.0/(1+L1+L2+R1+R2);        //差比和 (-100~100)
    Speed_bias=fabs(PlacePID_Control(&Turn_PID,ADC_bias,0.0,Turn_pd));   

    if(Speed_bias>10) Speed_bias=10;     //限制最大偏差

    if(ADC_bias<10&&ADC_bias>-10&&straight_count<20){           //在直道计数
        straight_count++;
        if(ADC_bias>=0){       //左转
            Left_High_Speed = basic_Speed - Speed_bias;
            Right_High_Speed = basic_Speed + Speed_bias*0.8;   //外轮加的少一点
        }
        else{               //右转
            Left_High_Speed = basic_Speed + Speed_bias*0.8;  
            Right_High_Speed = basic_Speed - Speed_bias;
        }
    }else if(ADC_bias>10||ADC_bias<-10){          //不在直道
        straight_count=0;
        if(ADC_bias>=0){       //左转
            Left_High_Speed = basic_Speed - Speed_bias;
            Right_High_Speed = basic_Speed + Speed_bias*0.8;   //外轮加的少一点
        }
        else{               //右转
            Left_High_Speed = basic_Speed + Speed_bias*0.8;  
            Right_High_Speed = basic_Speed - Speed_bias;
        }
    }else if(ADC_bias<10&&ADC_bias>-10&&straight_count>=20){    //判定在直道
        if(ADC_bias>=0){       //左转
            Left_High_Speed = straight_speed - Speed_bias;
            Right_High_Speed = straight_speed + Speed_bias*0.8;   //外轮加的少一点
        }
        else{               //右转
            Left_High_Speed = straight_speed + Speed_bias*0.8;  
            Right_High_Speed = straight_speed - Speed_bias;
        }
    }
    if(outtrack_flag){
        Left_High_Speed=0;
        Right_High_Speed=0;
    }

    encoder_L = -encoder_get_count(ENCODER_DIR_1);                     //获取编码器计数(编码器脉冲数直接反映了电机轴的转动量)
    encoder_R = encoder_get_count(ENCODER_DIR_2);             

    L_MOTOR_Duty += SpeedPID_Control(&Left_MOTOR_PID, encoder_L, Left_High_Speed, L_MOTOR_PID);
	R_MOTOR_Duty += SpeedPID_Control(&Right_MOTOR_PID, encoder_R, Right_High_Speed, R_MOTOR_PID);

    L_MOTOR_Duty=motor_limit(L_MOTOR_Duty);
    R_MOTOR_Duty=motor_limit(R_MOTOR_Duty);

    pwm_set_duty(PWM_L1,(uint32)(L_MOTOR_Duty * (PWM_DUTY_MAX / 100)));         //设置电机占空比         
    pwm_set_duty(PWM_L2, 0);                                            

    pwm_set_duty(PWM_R1,(uint32)(R_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                    
    pwm_set_duty(PWM_R2, 0);                                                                              

    encoder_clear_count(ENCODER_DIR_1);                                		   //清空编码器计数
    encoder_clear_count(ENCODER_DIR_2);    
    
    if(time_count==50){
        sprintf((char *)data_buffer,"%.2f %.2f %.2f %.2f\n",ADC_bias,Speed_bias,Left_High_Speed,Right_High_Speed);
        data_len = strlen(data_buffer);
        wireless_uart_send_buffer(data_buffer, data_len);
        time_count=0;
    }
}

double PlacePID_Control(PID*p, double Now_bias, double Set_bias, double *Turn_pd)    //???PD,????????
{
	double Output;  
	
	p->KP = *Turn_pd;     
	p->KD = *(Turn_pd+1);
	
	p->iError = Set_bias - Now_bias;  
	
	Output = p->KP * p->iError  
		   + p->KD * (p->iError - p->LastError);  
	
	p->LastError = p->iError;  
	
	return Output;
}

double SpeedPID_Control(PID*p, double ActualSpeed, double SetSpeed, double *MOTOR_PID)
{
	double Increase;    
	p->KP = *MOTOR_PID;    //参数赋值
	p->KI = *(MOTOR_PID+1);
	p->KD = *(MOTOR_PID+2);

	p->iError = SetSpeed - ActualSpeed;  

    p->iError = (p->iError>100)?100:((p->iError<-100)?-100:p->iError);       //积分项的限幅处理

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
