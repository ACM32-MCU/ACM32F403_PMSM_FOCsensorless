//UTF8
#include "bsp_advance_tim.h"


TIM_HandleTypeDef TIM1_Handler;


// //在高电平中心处触发Update中断，GPIO15翻转用于演示
// void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
// {
//     if (TIM1->SR & TIMER_SR_UIF)
//     {
//         GPIOAB->ODATA ^= (1<<15);
//     }
    
//     TIM1->SR = 0;   //write 0 to clear hardware flag      
// }


//6路互补PWM的GPIO初始化
void Init_Gpio_TIM1_PWM(void)  
{
    GPIO_InitTypeDef gpio_init; 	
    TIM_OC_InitTypeDef Tim_OC_Init_Para;
    TIM_BreakDeadTimeConfigTypeDef Tim_BDTR_Init_Para;
    TIM_MasterConfigTypeDef sMasterConfig;
    uint32_t timer_clock;

    /* GPIO初始化 */
    /* PA8  : TIM1_CH1  */
    /* PA7  : TIM1_CH1N */
    /* PA9  : TIM1_CH2  */
    /* PB0  : TIM1_CH2N */
    /* PA10 : TIM1_CH3  */
    /* PB1  : TIM1_CH3N */    
    gpio_init.Pin = GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;  
    gpio_init.Mode = GPIO_MODE_AF_PP;          
    gpio_init.Pull = GPIO_NOPULL;    
    gpio_init.Alternate = GPIO_FUNCTION_3;         
    HAL_GPIO_Init(GPIOA, &gpio_init);

    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;  
    gpio_init.Mode = GPIO_MODE_AF_PP;          
    gpio_init.Pull = GPIO_NOPULL;    
    gpio_init.Alternate = GPIO_FUNCTION_3;         
    HAL_GPIO_Init(GPIOB, &gpio_init);     	  

    /* 复位TIM1模块，并使能NVIC TIM1中断。ACM32F4只有3Bits优先级，配置为全部用于抢占优先级。 */
    System_Module_Reset(RST_TIM1);   
    System_Module_Enable(EN_TIM1);  
    NVIC_ClearPendingIRQ(TIM1_BRK_UP_TRG_COM_IRQn);     
    NVIC_EnableIRQ(TIM1_BRK_UP_TRG_COM_IRQn);
    NVIC_SetPriorityGrouping(__NVIC_PRIO_BITS); 

    /* 如果PCLK != HCLK， TIM1源时钟等于PCLK*2 */
    timer_clock = System_Get_APBClock();    
    if (System_Get_SystemClock() != System_Get_APBClock())
    {
        timer_clock =  System_Get_APBClock() << 1;    
    }

    /* TIM1基础定时器配置 */
    TIM1_Handler.Instance = TIM1;
    /* 使能预加载 */
    TIM1_Handler.Init.ARRPreLoadEn = TIM_ARR_PRELOAD_ENABLE;
    /* 设置死区和滤波采样时钟与定时器时钟(CK_INT)的比例 */      
    TIM1_Handler.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    /* 中心对齐计数模式1 */
    TIM1_Handler.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; 
    /* 重复计数模式关闭 */
    TIM1_Handler.Init.RepetitionCounter = ADVANCED_TIM_REPETITIONCOUNTER;  
    /* 预分频因子 */
    TIM1_Handler.Init.Prescaler = (timer_clock/ADVANCED_TIM_FREQ) - 1; 
    /* CNT周期 */
    TIM1_Handler.Init.Period = ADVANCED_TIM_PERIOD;
    HAL_TIMER_Base_Init(&TIM1_Handler);  

    /* 将TIM1 OC4REF配置为TRGO */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMER_Master_Mode_Config(TIM1_Handler.Instance, &sMasterConfig);

    /* 设置为PWM模式1 */
    Tim_OC_Init_Para.OCMode = OUTPUT_MODE_PWM1; 
    /* OCx和OCxN死区输出状态 */ 
    Tim_OC_Init_Para.OCIdleState = OUTPUT_IDLE_STATE_0;
    Tim_OC_Init_Para.OCNIdleState = OUTPUT_IDLE_STATE_0;  
    /* OCx和OCxN高电平有效 */
    Tim_OC_Init_Para.OCPolarity = OUTPUT_POL_ACTIVE_HIGH;  
    Tim_OC_Init_Para.OCNPolarity = OUTPUT_POL_ACTIVE_LOW; 
    /* 禁止快速模式 */
    Tim_OC_Init_Para.OCFastMode =  OUTPUT_FAST_MODE_DISABLE;  
    /* 设置脉冲宽度(占空比) */
    Tim_OC_Init_Para.Pulse = 0;

    /* 配置CH1/2/3 */
    HAL_TIMER_Output_Config(TIM1_Handler.Instance, &Tim_OC_Init_Para, TIM_CHANNEL_1); 
    HAL_TIMER_Output_Config(TIM1_Handler.Instance, &Tim_OC_Init_Para, TIM_CHANNEL_2);
    HAL_TIMER_Output_Config(TIM1_Handler.Instance, &Tim_OC_Init_Para, TIM_CHANNEL_3);
    /* 禁止OC1/2/3和OC1/2/3N预加载功能 */
    TIM1_Handler.Instance->CCMR1 &=  ~(BIT3 | BIT11);
    TIM1_Handler.Instance->CCMR2 &=  ~(BIT3);

    /* CH4配置 */
    Tim_OC_Init_Para.OCMode = OUTPUT_MODE_PWM2;  
    // Tim_OC_Init_Para.OCIdleState = OUTPUT_IDLE_STATE_0;
    // Tim_OC_Init_Para.OCNIdleState = OUTPUT_IDLE_STATE_0;      
    // Tim_OC_Init_Para.OCPolarity = OUTPUT_POL_ACTIVE_HIGH;  
    Tim_OC_Init_Para.OCNPolarity = OUTPUT_POL_ACTIVE_HIGH;    
    // Tim_OC_Init_Para.OCFastMode =  OUTPUT_FAST_MODE_DISABLE;  
    Tim_OC_Init_Para.Pulse = 3500; 	
    HAL_TIMER_Output_Config(TIM1_Handler.Instance, &Tim_OC_Init_Para, TIM_CHANNEL_4);  
    /* 开启OC4 */
    TIM1_Handler.Instance->CCER |=  BIT12; 

    /* 刹车配置 */	
    /* 使能OSSI和OSSR */
    Tim_BDTR_Init_Para.OffStateRunMode = 1;
    Tim_BDTR_Init_Para.OffStateIDLEMode = 1;
    /* Lock Level = 1 */
    Tim_BDTR_Init_Para.LockLevel = 1;
    /* 设置死区时间 */
    Tim_BDTR_Init_Para.DeadTime = DEADTIME;
    /* 刹车使能 */
    Tim_BDTR_Init_Para.BreakState = 1;
    /* 刹车输入电平极性，高有效 */
    Tim_BDTR_Init_Para.BreakPolarity = 1;
    /* 禁止自动输出 */
    Tim_BDTR_Init_Para.AutomaticOutput = 1;
    /* 刹车滤波，无滤波 */
    Tim_BDTR_Init_Para.BreakFilter = 0;

    HAL_TIMEx_ConfigBreakDeadTime(&TIM1_Handler, &Tim_BDTR_Init_Para);

    /* 中断配置 */
    /* 清除刹车、更新中断状态，并使能中断 */
    // TIM1_Handler.Instance->SR = ~(TIMER_SR_UIF | TIMER_SR_BIF);
    // TIM1_Handler.Instance->DIER |= (TIM_IT_UPDATE | TIM_IT_BREAK);
    TIM1_Handler.Instance->SR = ~(TIMER_SR_BIF);
    TIM1_Handler.Instance->DIER |= (TIM_IT_BREAK);


    /* 启动计数器，置位CNE */
    TIM1_Handler.Instance->CR1 |= BIT0;
    /* 启动PWM，置位MOE */
    TIM1_Handler.Instance->BDTR |= TIM_BDTR_MOE;

    /* TIM1中断优先级配置为1 */
    NVIC_SetPriority(TIM1_BRK_UP_TRG_COM_IRQn, 1);
}

// stop mcu pwm output.
void StopPWM(void)
{
    TIM1_Handler.Instance->CCER &= ~(BIT0 | BIT2 | BIT4 | BIT6 | BIT8 | BIT10);
}

// start mcu pwm output.
void StartPWM(void)
{
    /* 开启OC1/2/3和OC1/2/3N */
    TIM1_Handler.Instance->CCER |= (BIT0 | BIT2 | BIT4 | BIT6 | BIT8 | BIT10);
}		

