#include "zf_common_headfile.h"

#define PIT_CH                          (TIM1_PIT )                             //????????? 

#define ENCODER_DIR_1                	(TIM3_ENCOEDER)                         //??????????
#define ENCODER_DIR_DIR_1           	(IO_P13)             					
#define ENCODER_DIR_PULSE_1       		(TIM3_ENCOEDER_P04)   

#define ENCODER_DIR_2                 	(TIM0_ENCOEDER)                         
#define ENCODER_DIR_DIR_2              	(IO_P35)            				 	
#define ENCODER_DIR_PULSE_2            	(TIM0_ENCOEDER_P34)     

int16 encoder_L = 0;                                                   //???????????
int16 encoder_R = 0;

#define PWM_L1              (PWMB_CH1_P50)                                      //????1?2??
#define PWM_L2              (PWMB_CH4_P53)

#define PWM_R1              (PWMB_CH2_P51)
#define PWM_R2              (PWMB_CH3_P52)

#define MAX_DUTY            (50)                //???????50%

double L_MOTOR_Duty=30, R_MOTOR_Duty=30;	

void pit_handler (void);

void main()
{
    clock_init(SYSTEM_CLOCK_40M);
	debug_init();								

    encoder_dir_init(ENCODER_DIR_1, ENCODER_DIR_DIR_1, ENCODER_DIR_PULSE_1);   	//??????
    encoder_dir_init(ENCODER_DIR_2, ENCODER_DIR_DIR_2, ENCODER_DIR_PULSE_2);    
		
	pwm_init(PWM_L1, 17000, 0);                                                 //?????????0
    pwm_init(PWM_L2, 17000, 0);                                                 
    pwm_init(PWM_R1, 17000, 0);                                                 
    pwm_init(PWM_R2, 17000, 0);                                                 
	
	tim1_irq_handler = pit_handler;                                             //????????
	
    pit_ms_init(PIT_CH, 100);                                                   //????????100ms
	
    
	while(1)
    {
			pwm_set_duty(PWM_L1, L_MOTOR_Duty * (PWM_DUTY_MAX / 100));         //???????         
			pwm_set_duty(PWM_L2, 0);                                            

			pwm_set_duty(PWM_R1, R_MOTOR_Duty * (PWM_DUTY_MAX / 100));                  
			pwm_set_duty(PWM_R2, 0);                                                                              

    }
}

void pit_handler (void)
{
    encoder_L = -encoder_get_count(ENCODER_DIR_1);                     //???????(??????????????????)
    encoder_R = encoder_get_count(ENCODER_DIR_2);      

		printf("L=%d\nR=%d\n",encoder_L,encoder_R);

    encoder_clear_count(ENCODER_DIR_1);                                		   //???????
    encoder_clear_count(ENCODER_DIR_2);                             		
}


