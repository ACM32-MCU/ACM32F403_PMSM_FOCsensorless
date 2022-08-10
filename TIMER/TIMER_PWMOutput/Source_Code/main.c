/*
  ******************************************************************************
  * @file    main.c
  * @brief   main source File.
  ******************************************************************************
*/
//#include "app.h"
#include "bsp_adc.h"
#include "bsp_advance_tim.h"

#define UART_BAUD_RATE  115200

#define FILTER_PERIOD   20

extern stMcInfo mc_info;

UART_HandleTypeDef UART2_Handle;  

uint32_t ADC_VrefP = 0;


enum
{
    BUTTON_RELEASE,
    BUTTON_PRESSING,
    BUTTON_LONG_PRESS,
    BUTTON_SHORT_PRESS
    
};

enum
{
    FILTER_IDLE,
    FILTER_BUSY
};

enum
{
    SPEED_DIR_UP,
    SPEED_DIR_DOWN
};

/* 设置按键滤波时间，以systick为单位，systick默认初始化为1ms */
volatile uint32_t FilterCnt = 0;

/* 按键滤波阶段 */
uint16_t FilterStage = FILTER_IDLE;
/* 上一次有效的按键状态 */
uint16_t lastState = BUTTON_RELEASE;
/* 调速方向, 0:向上；1：向下 */
uint16_t speedDir = SPEED_DIR_UP;


/************************************************************************
 * function   : Uart_Init
 * Description: Uart Initiation. 
 ************************************************************************/ 
void Uart_Init(void) 
{
    UART2_Handle.Instance        = UART2;    
    UART2_Handle.Init.BaudRate   = UART_BAUD_RATE; 
    UART2_Handle.Init.WordLength = UART_WORDLENGTH_8B;
    UART2_Handle.Init.StopBits   = UART_STOPBITS_1;
    UART2_Handle.Init.Parity     = UART_PARITY_NONE;
    UART2_Handle.Init.Mode       = UART_MODE_TX_RX_DEBUG;
    UART2_Handle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    
    HAL_UART_Init(&UART2_Handle);     
    
    // UART_DEBUG_ENABLE control printfS   
    printfS("MCU is running, HCLK=%dHz, PCLK=%dHz\n", System_Get_SystemClock(), System_Get_APBClock());  
}

 //PA5用于观察ADC DMA转换完成中断
 //PA15用于观察TIM1 Update中断，看到是否发生在PWM高电平中心处
 void Test_GPIO_Init(void)
 {
     GPIO_InitTypeDef gpio_init; 

     gpio_init.Pin = GPIO_PIN_5 | GPIO_PIN_15;  
     gpio_init.Mode = GPIO_MODE_OUTPUT_PP;          
     gpio_init.Pull = GPIO_NOPULL;    
     gpio_init.Alternate = GPIO_FUNCTION_0;         
     HAL_GPIO_Init(GPIOA, &gpio_init);
     
     GPIOAB->ODATA |= (1<<15);
 }

/* 调节PWM的按键为PC13，PC13为特殊引脚，需通过RPMU模块进行设置 */
void PWMButtonGPIOInit(void)
{
    GPIO_InitTypeDef GPIOC_Handle; 

    GPIOC_Handle.Pin       = GPIO_PIN_13;
    GPIOC_Handle.Mode      = GPIO_MODE_INPUT;  
    GPIOC_Handle.Alternate = GPIO_FUNCTION_0;

    HAL_GPIO_Init(GPIOC, &GPIOC_Handle);

    /* RTC access enable */
    System_Enable_Disable_RTC_Domain_Access(FUNC_ENABLE);  
    
    __HAL_RTC_PC13_SEL(0);  // GPIO function   
    __HAL_RTC_PC13_PULL_UP_ENABLE();
    __HAL_RTC_PC13_DIGIT();  
}

/* 获取PC13状态，1：高电平，0：低电平 */ 
int GetPWMButtonState(void)
{
    return ((GPIOCD->IDATA & GPIO_PIN_13) >> 13);
}



/*********************************************************************************
* Function    : main
* Description : 
* Input       : 
* Outpu       : 
* Author      : xwl                         Data : 2020年
**********************************************************************************/
int main(void)
{
    System_Init(); 
    Uart_Init();  
	
    //Test_GPIO_Init(); 
    PWMButtonGPIOInit();
    ADC_VrefP = ADC_GetVrefP(ADC_CHANNEL_8);
    ADC_Motor();
    Init_Gpio_TIM1_PWM();
    StartPWM();

    while(1)
    {     
        if(GetPWMButtonState() == 0) //检测到按键按下
        {
            if(lastState == BUTTON_PRESSING)
            {
                //长按,按需添加逻辑
                continue;
            }

            if(FilterStage == FILTER_IDLE)
            {
                FilterCnt = FILTER_PERIOD;
                FilterStage = FILTER_BUSY;
            }
            else
            {
                if(FilterCnt == 0)
                {
                    FilterStage = FILTER_IDLE;
                    lastState = BUTTON_PRESSING;
                    
                    //GPIOAB->ODATA &= ~(1<<15);
                }
            }           
        }
        else //检测到按键未按下
        {
            if(lastState == BUTTON_PRESSING) //按键释放滤波
            {
                if(FilterStage == FILTER_IDLE)
                {
                    FilterCnt = FILTER_PERIOD;
                    FilterStage = FILTER_BUSY;
                }
                else
                {
                    if(FilterCnt == 0)
                    {
                        FilterStage = FILTER_IDLE;
                        lastState = BUTTON_RELEASE;
                        
                        //GPIOAB->ODATA |= (1<<15);

                        if(mc_info.refRPM == 0)
                        {
                            mc_info.refRPM =200; //启动
                            speedDir = SPEED_DIR_UP;
                            continue;
                        }

                        if(mc_info.refRPM == 2800)
                        {
                            mc_info.refRPM = 2796; 
                            speedDir = SPEED_DIR_DOWN;
                            continue;
                        }

                        if(speedDir == SPEED_DIR_UP)
                        {
                            mc_info.refRPM += 100;
                        }

                        if(speedDir == SPEED_DIR_DOWN)
                        {
                            if(mc_info.refRPM > 200)
                                mc_info.refRPM -= 100;
                            else 
                                mc_info.refRPM = 0;
                        }
                    }
                }  
            }
            else
            {
                FilterStage = FILTER_IDLE;
            }
        }

    }
}

