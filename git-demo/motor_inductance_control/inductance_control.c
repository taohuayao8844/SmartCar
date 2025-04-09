#include "zf_common_headfile.h"
#include "math.h"
#include "string.h"

#define PIT_CH                  (TIM1_PIT)                             //使用的周期中断编号 

#define ADC_CHANNEL1            (ADC_CH14_P06)                         //定义ADC通道
#define ADC_CHANNEL2            (ADC_CH13_P05)
#define ADC_CHANNEL3            (ADC_CH9_P01)
#define ADC_CHANNEL4            (ADC_CH0_P10)

int L1=0, L2=0, R2=0, R1=0;				    //四路电感值
double ADC_bias=0;                    //差比和偏差值
double Left_High_Speed=0,Right_High_Speed=0,Speed_bias=0;  //左右电机目标速度,速度偏差调整
double basic_Speed=30,straight_speed=40;           //基础速度，直线速度
int outtrack_flag=0;                //冲出赛道标志
int straight_count=0;               //直线计数
uint16 adc_max[4]={950,940,950,950};             //存储各电感最大值

typedef struct PID
{
	double iError;  
    double LastError;  
    double PrevError;  
    double KP;
    double KI;
    double KD;
}PID;
PID Turn_PID;                       //转向控制PID
double Turn_pd[2]={0.6,0.2};        //PD参数

double PlacePID_Control(PID*sptr, double Now_bias, double Set_bias, double *Turn_pd); 

uint8 data_buffer[32];                                    //无线串口通信
uint8 data_len = 0;
uint32 time_count = 0;

void pit_handler (void); 

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

    adc_init(ADC_CHANNEL1, ADC_10BIT);                                          
    adc_init(ADC_CHANNEL2, ADC_10BIT);                                          
    adc_init(ADC_CHANNEL3, ADC_10BIT);                                         
    adc_init(ADC_CHANNEL4, ADC_10BIT);

    memset(&Turn_PID,0,sizeof(Turn_PID));        //结构体赋初值

	tim1_irq_handler = pit_handler; 

	wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              //无线串口输出测试信息
    
    pit_ms_init(PIT_CH, 10);    

	while(1)
    {
		
    }
}



void pit_handler (void){

    time_count++;
    L1=adc_mean_filter_convert(ADC_CHANNEL1,5);      //5次均值滤波转换结果
    L2=adc_mean_filter_convert(ADC_CHANNEL2,5);      
    R2=adc_mean_filter_convert(ADC_CHANNEL3,5);    
    R1=adc_mean_filter_convert(ADC_CHANNEL4,5);

    L1=(unsigned long)L1*100/adc_max[0];           //归一化  (0-100)
    L2=(unsigned long)L2*100/adc_max[1];
    R2=(unsigned long)R2*100/adc_max[2];
    R1=(unsigned long)R1*100/adc_max[3];

    if(L1<10&&L2<10&&R2<10&&R1<10)          //冲出赛道保护
        outtrack_flag=1;
    else
        outtrack_flag=0;

    ADC_bias=(double)((L1 - R1)+(L2 - R2))*100.0/(1+L1+L2+R1+R2);        //差比和 (-100~100)
    Speed_bias=fabs(PlacePID_Control(&Turn_PID,ADC_bias,0.0,Turn_pd));   

    if(Speed_bias>20) Speed_bias=20;     //限制最大偏差

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

    if(time_count==50){
        sprintf((char *)data_buffer, "%d %d %.2f %.2f\n",L1,R1,ADC_bias,Speed_bias);
        data_len = strlen(data_buffer);
        wireless_uart_send_buffer(data_buffer, data_len);
        time_count=0;
    }
}

double PlacePID_Control(PID*sptr, double Now_bias, double Set_bias, double *Turn_pd)    //位置式PD，计算给电机的偏差
{
	double Output;  
	
	sptr->KP = *Turn_pd;     
	sptr->KD = *(Turn_pd+1);
	
	sptr->iError = Set_bias - Now_bias;  
	
	Output = sptr->KP * sptr->iError  
		   + sptr->KD * (sptr->iError - sptr->LastError);  
	
	sptr->LastError = sptr->iError;  
	
	return Output;
}