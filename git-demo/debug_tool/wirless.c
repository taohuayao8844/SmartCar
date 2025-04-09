#include "zf_common_headfile.h"
#include "string.h"
#define PIT_CH                          (TIM1_PIT )                             //????????? 

uint8 data_buffer[32];
uint8 data_len;
uint32 time_count = 0;

#define ENCODER_DIR_1                	(TIM3_ENCOEDER)                         //??????????
#define ENCODER_DIR_DIR_1           	(IO_P13)             					
#define ENCODER_DIR_PULSE_1       		(TIM3_ENCOEDER_P04)   

#define ENCODER_DIR_2                 	(TIM0_ENCOEDER)                         
#define ENCODER_DIR_DIR_2              	(IO_P35)            				 	
#define ENCODER_DIR_PULSE_2            	(TIM0_ENCOEDER_P34)            			         		

int16 encoder_L = 0;                                                   //???????????
int16 encoder_R = 0;

#define PWM_L1              (PWMB_CH1_P50)                                      //?????????
#define PWM_L2              (PWMB_CH4_P53)

#define PWM_R1              (PWMB_CH2_P51)
#define PWM_R2              (PWMB_CH3_P52)

#define MAX_DUTY            (30)                //???????50%

double L_MOTOR_Duty=18, R_MOTOR_Duty=18;				//初始为10或0差不多同时转,15就左轮先转了

typedef struct PID                             //PID???
{
	double iError;     //????
    double LastError;  //????
    double PrevError;  //?????
    double KP;
    double KI;
    double KD;
}PID;

PID Left_MOTOR_PID, Right_MOTOR_PID;                //???PID
double L_MOTOR_PID[3] = {0.005, 0.0012, 0.0025};                  //??????PID??
double R_MOTOR_PID[3] = {0.004, 0.0015, 0.002};
double Left_High_Speed = 150, Right_High_Speed = 150;           //???????????

void pit_handler (void);                                                        //????????
double PID_speed(PID*p, double ActualSpeed, double SetSpeed, double *MOTOR_PID);        //???PID(???????????)
double motor_limit(double duty);                          //??????

void main()
{
    clock_init(SYSTEM_CLOCK_40M);
	debug_init();								

	if(wireless_uart_init())                                                    // ÅÐ¶ÏÊÇ·ñÍ¨¹ý³õÊ¼»¯
    {
        while(1)                                                                // ³õÊ¼»¯Ê§°Ü¾ÍÔÚÕâ½øÈëËÀÑ­»·
        {
            
        }
    }
	
    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);   	//??????
    encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);    
		
	pwm_init(PWM_L1, 17000,(uint32)(L_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                                               //?????????0
    pwm_init(PWM_L2, 17000, 0);                                                 
    pwm_init(PWM_R1, 17000,(uint32)(R_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                                              
    pwm_init(PWM_R2, 17000, 0);                                                 
	
	tim1_irq_handler = pit_handler;                                             //????????
	
	wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
	wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // ³õÊ¼»¯Õý³£ Êä³ö²âÊÔÐÅÏ¢
	
    memset(&Left_MOTOR_PID, 0, sizeof(Left_MOTOR_PID));
	memset(&Right_MOTOR_PID, 0, sizeof(Right_MOTOR_PID));

    pit_ms_init(PIT_CH, 10);                                                   //????????100ms

	while(1)
    {
			                                            
		
    }
}

void pit_handler (void)
{
	time_count++;
    encoder_L = -encoder_get_count(ENCODER_DIR_1);                     //???????(??????????????????)
    encoder_R = encoder_get_count(ENCODER_DIR_2);             

    L_MOTOR_Duty += PID_speed(&Left_MOTOR_PID, encoder_L, Left_High_Speed, L_MOTOR_PID);
	R_MOTOR_Duty += PID_speed(&Right_MOTOR_PID, encoder_R, Right_High_Speed, R_MOTOR_PID);

    L_MOTOR_Duty=motor_limit(L_MOTOR_Duty);
    R_MOTOR_Duty=motor_limit(R_MOTOR_Duty);

    pwm_set_duty(PWM_L1, (uint32)(L_MOTOR_Duty * (PWM_DUTY_MAX / 100)));         //???????         
    pwm_set_duty(PWM_L2, 0);                                            

    pwm_set_duty(PWM_R1,(uint32)(R_MOTOR_Duty * (PWM_DUTY_MAX / 100)));                  
    pwm_set_duty(PWM_R2, 0);                                                                              

    encoder_clear_count(ENCODER_DIR_1);                                		   //???????
    encoder_clear_count(ENCODER_DIR_2);     

    if(time_count==100){
        sprintf((char *)data_buffer, "Lduty=%.2f Rduty=%.2f %d %d",L_MOTOR_Duty,R_MOTOR_Duty,encoder_L,encoder_R);
        data_len = strlen(data_buffer);
        wireless_uart_send_buffer(data_buffer, data_len);
        time_count=0;
    }
}

double PID_speed(PID*p, double ActualSpeed, double SetSpeed, double *MOTOR_PID)
{
	double Increase;    
	p->KP = *MOTOR_PID;    //pid参数的赋值
	p->KI = *(MOTOR_PID+1);
	p->KD = *(MOTOR_PID+2);

	p->iError = SetSpeed - ActualSpeed;  

    p->iError = (p->iError>100)?50:((p->iError<-50)?-50:p->iError); // 积分项的限幅处理

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
