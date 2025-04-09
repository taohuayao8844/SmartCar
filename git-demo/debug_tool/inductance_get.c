#include "zf_common_headfile.h"

uint8 data_buffer[32];
uint8 data_len;
uint32 time_count = 0;

#define PIT_CH                          (TIM1_PIT)                             //????????? 

#define ADC_CHANNEL1            (ADC_CH14_P06)
#define ADC_CHANNEL2            (ADC_CH13_P05)
#define ADC_CHANNEL3            (ADC_CH9_P01)
#define ADC_CHANNEL4            (ADC_CH0_P10)

int L1, L2, R2, R1;				    //?????
double elevalue;                    //??????
double Left_High_Speed,Right_High_Speed,basic_Speed;  //???????,???

void pit_handler (void);  

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
    adc_init(ADC_CHANNEL1, ADC_10BIT);                                          
    adc_init(ADC_CHANNEL2, ADC_10BIT);                                          
    adc_init(ADC_CHANNEL3, ADC_10BIT);                                         
    adc_init(ADC_CHANNEL4, ADC_10BIT);                                          
		
		tim1_irq_handler = pit_handler;                                             
	
    pit_ms_init(PIT_CH, 10);    
		
		wireless_uart_send_byte('\r');
		wireless_uart_send_byte('\n');
		wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // ³õÊ¼»¯Õý³£ Êä³ö²âÊÔÐÅÏ¢
		
		while(1)
		{
				
		}
}



void pit_handler (void){
		time_count++;
    L1=adc_mean_filter_convert(ADC_CHANNEL1,5);      //5?????????
    L2=adc_mean_filter_convert(ADC_CHANNEL2,5);      
    R2=adc_mean_filter_convert(ADC_CHANNEL3,5);    
    R1=adc_mean_filter_convert(ADC_CHANNEL4,5);
		if(time_count==100){
				time_count=0;
				sprintf((char *)data_buffer, "L1=%d L2=%d R2=%d R1=%d\n",L1,L2,R2,R1);
				data_len = strlen(data_buffer);
				wireless_uart_send_buffer(data_buffer, data_len);
		}
}
