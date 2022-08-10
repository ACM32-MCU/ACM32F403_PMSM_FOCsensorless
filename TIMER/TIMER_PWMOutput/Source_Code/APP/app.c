/***********************************************************************
 * Filename    : app.c
 * Description : app source file
 * Author(s)   : xwl  
 * version     : V1.1
 * Modify date : 2021-04-07  
 ***********************************************************************/
#include  "app.h"
   
#define PWM_OUTPUT_CHANNEL1    TIM_CHANNEL_1   

#define TIM_CLOCK_FREQ            (8000000U)  

TIM_HandleTypeDef TIM_Handler;     

/*********************************************************************************
* Function    : TIM1_MSP_Pre_Init      
* Description : timer 1 initiation, includes clock, nvic 
* Input       : htim, timer handler     
* Output      : none 
* Author      : xwl                       
**********************************************************************************/ 
void TIM1_MSP_Pre_Init(TIM_HandleTypeDef * htim)   
{
	HAL_TIMER_MSP_Init(htim);      
} 


/*********************************************************************************
* Function    : TIM1_MSP_Post_Init      
* Description : timer 1 gpio initiation, configure PC8 as TIMER1 CH1 
* Input       : none     
* Output      : none 
* Author      : xwl                       
**********************************************************************************/ 
void TIM1_MSP_Post_Init(void)     
{
	GPIO_InitTypeDef gpio_init; 
	
	gpio_init.Pin = GPIO_PIN_8;  //TIM1_CH1  
	gpio_init.Mode = GPIO_MODE_AF_PP;          
	gpio_init.Pull = GPIO_NOPULL;    
	gpio_init.Alternate = GPIO_FUNCTION_2;         
	HAL_GPIO_Init(GPIOC, &gpio_init);    // PC8   	  
}  

/*********************************************************************************
* Function    : TIM1_Init      
* Description : timer 1 initiation, includes clock, io, configuration 
* Input       : none    
* Output      : none 
* Author      : xwl                       
**********************************************************************************/  
void TIM1_Init(void)  
{  
	TIM_OC_InitTypeDef Tim_OC_Init_Para;   
    uint32_t timer_clock;
    
    timer_clock = System_Get_APBClock(); 
    
    if (System_Get_SystemClock() != System_Get_APBClock())  // if hclk/pclk != 1, then timer clk = pclk * 2  
    {
       timer_clock =  System_Get_APBClock() << 1;    
    }
	
	TIM_Handler.Instance = TIM1;
	TIM_Handler.Init.ARRPreLoadEn = TIM_ARR_PRELOAD_ENABLE;        
	TIM_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; 
	TIM_Handler.Init.CounterMode = TIM_COUNTERMODE_UP; 
	TIM_Handler.Init.RepetitionCounter = 0;  
	TIM_Handler.Init.Prescaler = (timer_clock/TIM_CLOCK_FREQ) - 1; 
	if (timer_clock%TIM_CLOCK_FREQ > TIM_CLOCK_FREQ/2) 
	{
		TIM_Handler.Init.Prescaler = TIM_Handler.Init.Prescaler + 1;  
	}
	TIM_Handler.Init.Period = (TIM_CLOCK_FREQ/1000000)*10 - 1;  // period = 10us      
	
	TIM1_MSP_Pre_Init(&TIM_Handler);       
	HAL_TIMER_Base_Init(&TIM_Handler);  
	
	Tim_OC_Init_Para.OCMode = OUTPUT_MODE_PWM1;  
	Tim_OC_Init_Para.OCIdleState = OUTPUT_IDLE_STATE_0;
	Tim_OC_Init_Para.OCNIdleState = OUTPUT_IDLE_STATE_0;      
	Tim_OC_Init_Para.OCPolarity = OUTPUT_POL_ACTIVE_HIGH;  
	Tim_OC_Init_Para.OCNPolarity = OUTPUT_POL_ACTIVE_HIGH;    
	Tim_OC_Init_Para.OCFastMode =  OUTPUT_FAST_MODE_DISABLE;  
	Tim_OC_Init_Para.Pulse = (TIM_Handler.Init.Period + 1)/2;   // 50% duty cycle   	
	HAL_TIMER_Output_Config(TIM_Handler.Instance, &Tim_OC_Init_Para, PWM_OUTPUT_CHANNEL1);    
    
	TIM1_MSP_Post_Init();   
	          
}

/*********************************************************************************
* Function    : Timer_PWM_Output_Test      
* Description : test function ,output period=10us 50% duty PWM waveform  
* Input       : none    
* Output      : none 
* Author      : xwl                       
**********************************************************************************/  
void Timer_PWM_Output_Test(void)  
{
	TIM1_Init(); 
	HAL_TIM_PWM_Output_Start(TIM_Handler.Instance, PWM_OUTPUT_CHANNEL1);       
    
	while(1)
	{

	}
}


