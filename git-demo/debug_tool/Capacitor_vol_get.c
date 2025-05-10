#include "zf_common_headfile.h"

#define PIT_CH                          (TIM1_PIT)                             //????????? 

#define ADC_VOL                         (ADC_CH2_P12)
#define VREF 3.3                       //ADC的参考电压,决定了ADC能够测量的最大电压值
#define ADC_RESOLUTION 4096            //ADC分辨率(12位ADC的分辨率是4096)

unsigned int adc_value;
float voltage;
void pit_handler (void);  

void main()
{
    clock_init(SYSTEM_CLOCK_40M);
	debug_init();								

    adc_init(ADC_VOL, ADC_10BIT);                                          
    
	tim1_irq_handler = pit_handler;                                             
	
    pit_ms_init(PIT_CH, 100);
		while(1)
		{
				
		}
}

void pit_handler (void){
    adc_value=adc_convert(ADC_VOL,ADC_12BIT);      
    voltage = (adc_value * VREF) / ADC_RESOLUTION;   //将ADC值转换为电压
    voltage *=11;                                    //ADC检测端口输出的电压为电池电压的1/11(约有0.05V的偏差)
    printf("%.2f",voltage);
}
