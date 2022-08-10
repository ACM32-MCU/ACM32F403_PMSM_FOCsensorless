/*
  ******************************************************************************
  * @file    System_ACM32F4.c
  * @version V1.0.0
  * @date    2021 
  * @brief   System Source File, includes clock management, reset management 
  *          and IO configuration, ...   
  ******************************************************************************
*/
#include "ACM32Fxx_HAL.h"

uint32_t gu32_SystemClock;
uint32_t gu32_APBClock;      
uint32_t gu32_Clocksrc = CLK_SRC_RCH;    

/* System count in SysTick_Handler */
volatile uint32_t gu32_SystemCount;

extern volatile uint32_t FilterCnt;

/************************* Miscellaneous Configuration ************************/
/*!< Uncomment the following line if you need to relocate your vector Table in
     Internal SRAM. */
/* #define VECT_TAB_SRAM */
#define VECT_TAB_OFFSET  0x0U /*!< Vector Table base offset field.
                                   This value must be a multiple of 0x200. */
/******************************************************************************/

/*********************************************************************************
* Function    : HardFault_Handler
* Description : Hard Fault handle, while(1) loop, wait for debug 
* Input       : none 
* Output      : none   
* Author      : xwl                           
**********************************************************************************/  
void HardFault_Handler(void)
{
    while(1);   
} 

/*********************************************************************************
* Function    : SysTick_Handler
* Description : System tick handler
* Input       : none 
* Output      : none 
* Author      : Chris_Kyle                         
**********************************************************************************/
void SysTick_Handler(void)
{
    gu32_SystemCount++;

    if(FilterCnt) FilterCnt--;
}

/*********************************************************************************
* Function    : System_SysTick_Init
* Description : System Tick Init. Period is 1 ms
* Input       : none 
* Output      : none 
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_SysTick_Init(void)
{
    gu32_SystemCount = 0;   
    SysTick_Config(gu32_SystemClock / 1000);  //1ms/tick  
}


/*********************************************************************************
* Function    : System_SysTick_Off
* Description : Turn off System Tick  
* Input       : none 
* Output      : none 
* Author      : xwl                         
**********************************************************************************/
void System_SysTick_Off(void)  
{
    SysTick->CTRL = 0;   
}
/*********************************************************************************
* Function    : System_Init
* Description : Initialize the system clock, accelerate function and system tick.   
* Input       : none 
* Output      : none 
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Init(void)
{
    SCU->RCR |= SCU_RCR_REMAP_EN;   // always remap enabled  
    System_Set_Buzzer_Divider(80, FUNC_DISABLE); // disable clock out   
    /* 3 bits for pre-emption priority, 0 bits for subpriority */ 
    NVIC_SetPriorityGrouping(NVIC_PRIORITY_GROUP_3);    

    /* Initialize the system clock */
    if (false == System_Clock_Init(DEFAULT_SYSTEM_CLOCK))
    {
        while(1);  
    }
    
#if (__ACCELERATE_PRESENT == 1)
    System_EnableIAccelerate();
#endif
    
#if (__ACCELERATE_EH_PRESENT == 1)   
    System_EnableDAccelerate();
#endif  

#ifdef HAL_SYSTICK_ENABLED  // To activate macro in ACM32Fxx_HAL.h      
    System_SysTick_Init();
#endif
}

/*********************************************************************************
* Function    : System_Core_Config
* Description : configure FPU and vector table address  
* Input       : none  
* Output      : none   
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Core_Config(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    /* set CP10 and CP11 Full Access */
    SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));
#endif

    /* Configure the Vector Table location add offset address ------------------*/
#ifdef VECT_TAB_SRAM
    SCB->VTOR = SRAM_BASE | VECT_TAB_OFFSET;    /* Vector Table Relocation in Internal SRAM */
#else
    SCB->VTOR = EFLASH_BASE | VECT_TAB_OFFSET;  /* Vector Table Relocation in Internal FLASH */
#endif
    
    if (SCU->RCHCR & SCU_RCHCR_RCH_DIV)
    {
        SET_EFC_RD_WAIT(2);
        System_Delay(1);   
        SCU->RCHCR &= (~SCU_RCHCR_RCH_DIV);
        System_Delay(1); 
        while(!(SCU->RCHCR & SCU_RCHCR_RCHRDY));     
    }
}


/*********************************************************************************
  System_Clock_Map={PLL_SOURCE , F_SystemClock ,PLL_F,PLL_N,PLL_M }
* PLL_SOURCE：PLLCLK_SRC_RC4M   PLLCLK_SRC_XTH_8M   PLLCLK_SRC_XTH_12M 
* F_SystemClock : Clock eg:180000000   120000000
* PLL_F      : 0-63
* PLL_N      : 0-7
* PLL_M      : 0-15                     
**********************************************************************************/    
const uint32_t System_Clock_Map[][5] =//Fpll=(Fin* (PLL_F+12)/(PLL_N+1))/ (PLL_M+1)
{ 
    { PLLCLK_SRC_RC4M   ,  180000000, 33,0,0},   // Fpll=4*(33+12)/(0+1)/(0+1)=180
    { PLLCLK_SRC_RC4M   ,  120000000, 18,0,0},   // Fpll=4*(18+12)/(0+1)/(0+1)=120
    { PLLCLK_SRC_XTH_8M ,  180000000, 33,1,0},   // Fpll=8*(33+12)/(1+1)/(0+1)=180
    { PLLCLK_SRC_XTH_8M ,  120000000, 18,1,0},   // Fpll=8*(18+12)/(1+1)/(0+1)=120
    { PLLCLK_SRC_XTH_12M,  180000000, 18,1,0},   // Fpll=12*(18+12)/(1+1)/(0+1)=180
    { PLLCLK_SRC_XTH_12M,  120000000, 18,2,0},   // Fpll=12*(18+12)/(2+1)/(0+1)=120
    { 0xffffffff        ,        0 ,  0,0,0},    // end flag=0xffffffff   
};

/*********************************************************************************
* Function    : Set_Pll_Div
* Description : Set Pll Div
* Input       : fu32_Clock: System core clock frequency, measured as Hz   
                lu32_pll_src：PLL_SOURCE，can be PLLCLK_SRC_RC4M PLLCLK_SRC_XTH_8M PLLCLK_SRC_XTH_12M
* Output      : 0: success, other value: fail reason  
* Author      : cwt                         
**********************************************************************************/    
uint32_t Set_Pll_Div(uint32_t fu32_Clock,uint32_t lu32_pll_src)
{
    uint32_t i=0,u32_pll_F,u32_pll_N,u32_pll_M;
    uint32_t u32_timeout=0,u32_result=1;
    
     /* if pll_src is XTH,need waid XTH READY*/
    if(lu32_pll_src==PLLCLK_SRC_XTH_8M||lu32_pll_src==PLLCLK_SRC_XTH_12M)
    {
        SCU->XTHCR = SCU_XTHCR_XTH_EN | SCU_XTHCR_READYTIME_32768;
        while (0 == (SCU->XTHCR & SCU_XTHCR_XTHRDY))
        {     
            if (u32_timeout == SYSTEM_TIMEOUT)
            {
                lu32_pll_src = PLLCLK_SRC_RC4M;     
                gu32_Clocksrc = CLK_SRC_RCH;  
                SCU->RCHCR |= SCU_RCHCR_RCH_DIV;   
                while(!(SCU->RCHCR & SCU_RCHCR_RCHRDY));                  
                break;     
            }  
            u32_timeout++;    
        } 
        
        if (u32_timeout < SYSTEM_TIMEOUT) 
        {
            gu32_Clocksrc = CLK_SRC_XTH;  
        }
    }
    else /* if pll_src is RCH,need waid RCH READY*/
    {
        SCU->RCHCR |= SCU_RCHCR_RCH_DIV;   
        while(!(SCU->RCHCR & SCU_RCHCR_RCHRDY));  
        gu32_Clocksrc = CLK_SRC_RCH;  
    }
    
    /*Enable PLL*/
    SCU->PLLCR |=  SCU_PLLCR_PLL_EN; 
    SCU->PLLCR &= ~(SCU_PLLCR_PLL_SLEEP);
    while(!(SCU->PLLCR & SCU_PLLCR_PLL_FREE_RUN)); 
    
     /*SET PLL DIV*/
    for(i = 0; System_Clock_Map[i][0] != 0xffffffff; i++)
    {
        if(System_Clock_Map[i][0]==lu32_pll_src && System_Clock_Map[i][1]==fu32_Clock)
        {
            u32_pll_F=System_Clock_Map[i][2];
            u32_pll_N=System_Clock_Map[i][3];
            u32_pll_M=System_Clock_Map[i][4];
            
            SCU->PLLCR = (SCU->PLLCR &(~(0x1FFFFU << 3))) | (u32_pll_F << 3) | (u32_pll_N << 12) | (u32_pll_M << 16);   
            if((lu32_pll_src == PLLCLK_SRC_XTH_8M) || (lu32_pll_src == PLLCLK_SRC_XTH_12M))   
            {
                SCU->PLLCR = (SCU->PLLCR & (~(0x3U << 1)) ) | (2 << 1); // select XTH  
            }
            else
            {
               SCU->PLLCR = (SCU->PLLCR & (~(0x3U << 1)) );   // select RC64M/16    
            }
            
            SCU->PLLCR |= SCU_PLLCR_PLL_UPDATE_EN;  
            System_Delay(16); //delay 1us    
            while(!(SCU->PLLCR & (SCU_PLLCR_PLL_FREE_RUN) ) );  
            System_Delay(16); //delay 1us 
            SCU->CCR1 = SCU_CCR1_SYS_PLL;  // configure system clock as PLL clock     
            u32_result=0; 
            if((lu32_pll_src == PLLCLK_SRC_XTH_8M) || (lu32_pll_src == PLLCLK_SRC_XTH_12M))
            {
                SCU->RCHCR |= SCU_RCHCR_RCH_DIV;   
            }
            break;  
        }
    }
    
    return u32_result;    
}

/*********************************************************************************
* Function    : System_Clock_Init
* Description : Clock init
* Input       : fu32_Clock: System core clock frequency, measured as Hz, could be 32MHz, 64MHz,
                120MHz, 180Mhz                
* Output      : 0: success, other value: fail reason  
* Author      : xwl                         
**********************************************************************************/    
bool System_Clock_Init(uint32_t fu32_Clock)
{
    volatile uint32_t lu32_sysdiv, lu32_pclkdiv, lu32_timeout, lu32_pll_src, lu32_pclk_div_para,lu32_result;     
             
    lu32_result = 0;  
    lu32_pll_src = PLL_SOURCE_FROM;    
    
    SET_EFC_RD_WAIT(RD_WAIT_ENSURE_OK)     
    if(0 == (SCU->RCHCR & SCU_RCHCR_RCHRDY))  
    {
       SCU->RCHCR |=  SCU_RCHCR_RCH_EN;   
       while(0 == (SCU->RCHCR & SCU_RCHCR_RCHRDY));  // wait RCH ready         
    }
    SCU->CCR1 = 0;  // select RC64M as default  
         
    if (fu32_Clock <= 64000000)  
    {
        if ((SCU->RCHCR) & SCU_RCHCR_RCH_DIV)
        {
            SCU->RCHCR &= (~SCU_RCHCR_RCH_DIV);   
            while(0 == (SCU->RCHCR & SCU_RCHCR_RCHRDY));    
        }
        
        if (fu32_Clock == 32000000)
        {
            gu32_SystemClock = fu32_Clock;
            lu32_sysdiv = 2;  
            lu32_pclkdiv = 1;  // pclk = hclk 
        }
        else
        {
            gu32_SystemClock = 64000000;
            lu32_sysdiv = 1;  
            lu32_pclkdiv = 1;   
        }
                
        gu32_Clocksrc = CLK_SRC_RCH;  
    }
   
    else // select pll as system clock 
    {
        lu32_result=Set_Pll_Div(fu32_Clock,lu32_pll_src);
        lu32_sysdiv = 1;  
        lu32_pclkdiv = 2; // pclk = hclk/2   
        gu32_SystemClock = fu32_Clock;      
    }
    
    if (0 == lu32_result)
    {
        if (1 == lu32_pclkdiv)
        {
            lu32_pclk_div_para = 0;    
        }
        else if (2 == lu32_pclkdiv)
        {
            lu32_pclk_div_para = 4;    // pclk = hclk/2   
        } 
        else
        {
            lu32_pclk_div_para = 5;    // pclk = hclk/4       
        }
    }
    else
    {
        gu32_SystemClock = 64000000;  
        lu32_sysdiv = 1; 
        lu32_pclkdiv = 1;  
        lu32_pclk_div_para = 0;  
        SCU->RCHCR &= (~SCU_RCHCR_RCH_DIV);    
        while(!(SCU->RCHCR & SCU_RCHCR_RCHRDY));  
        gu32_Clocksrc = CLK_SRC_RCH;  
    }
    
    gu32_APBClock = gu32_SystemClock/lu32_pclkdiv;         
  
    SCU->CCR2 = (SCU->CCR2 & (~0x7FFU)) | (lu32_sysdiv-1) | (lu32_pclk_div_para << 8);   
    while((SCU->CCR2 & SCU_CCR2_DIVDONE) == 0x00);  // wait divide done   

    HAL_EFlash_Init(gu32_SystemClock);    
    
    return (lu32_result == 0);    
}

/*********************************************************************************
* Function    : System_Get_SystemClock
* Description : get AHB clock frequency   
* Input       : none 
* Output      : frequency, measured as Hz    
* Author      : Chris_Kyle                         
**********************************************************************************/
uint32_t System_Get_SystemClock(void)
{
    return gu32_SystemClock;
}

/*********************************************************************************
* Function    : System_Get_APBClock
* Description : get APB clock frequency   
* Input       : none 
* Output      : frequency, measured as Hz  
* Author      : Chris_Kyle                         
**********************************************************************************/
uint32_t System_Get_APBClock(void)
{
    return gu32_APBClock;
}

/*********************************************************************************
* Function    : System_Get_Clk_Source
* Description : get clock source  
* Input       : none 
* Output      : CLK_SRC_RCH or CLK_SRC_XTH   
* Author      : xwl                          
**********************************************************************************/
uint32_t System_Get_Clk_Source(void)
{
    return gu32_Clocksrc;
}

/*********************************************************************************
* Function    : System_Module_Reset
* Description : reset module 
* Input       : module id 
* Output      : none 
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Module_Reset(enum_RST_ID_t fe_ID_Index)
{
    if (fe_ID_Index > 31) 
    {
        SCU->IPRST2 &= ~(1 << (fe_ID_Index - 32));         
        System_Delay(5);
        SCU->IPRST2 |= 1 << (fe_ID_Index - 32);   
    }
    else 
    {
        SCU->IPRST1 &= ~(1 << fe_ID_Index);       
        System_Delay(5);  
        SCU->IPRST1 |= 1 << fe_ID_Index;       
    }
}

/*********************************************************************************
* Function    : System_Module_Enable
* Description : enable module clock  
* Input       : module id     
* Output      : none 
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Module_Enable(enum_Enable_ID_t fe_ID_Index)
{
    if (fe_ID_Index > 13) 
    {
        SCU->IPCKENR1 |= 1 << (fe_ID_Index - 14);
    }
    else 
    {
        SCU->IPCKENR2 |= 1 << fe_ID_Index;
    }
    
    System_Delay(2);   
}

/*********************************************************************************
* Function    : System_Module_Disable
* Description : disable module clock 
* Input       : module id  
* Output      : none 
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Module_Disable(enum_Enable_ID_t fe_ID_Index)
{
    if (fe_ID_Index > 13) 
    {
        SCU->IPCKENR1 &= ~(1 << (fe_ID_Index - 14));
    }
    else 
    {
        SCU->IPCKENR2 &= ~(1 << fe_ID_Index);
    }
}

/*********************************************************************************
* Function    : System_Delay
* Description : NOP delay
* Input       : count 
* Output      : none  
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Delay(volatile uint32_t fu32_Delay)
{
    while (fu32_Delay--);
}

/*********************************************************************************
* Function    : System_Delay_MS
* Description : ms delay. Use this Function must call System_SysTick_Init()
* Input       : delay period, measured as ms  
* Output      : none  
* Author      : Chris_Kyle                         
**********************************************************************************/
void System_Delay_MS(volatile uint32_t fu32_Delay)
{
    uint32_t lu32_SystemCountBackup;
    
    lu32_SystemCountBackup = gu32_SystemCount;
    
    while ( (gu32_SystemCount - lu32_SystemCountBackup) < fu32_Delay);  
}

/*********************************************************************************
* Function    : System_Enable_RC32K
* Description : Enable RC32K, make sure RTC Domain Access is allowed    
* Input       : none 
* Output      : none 
* Author      : CWT                         
**********************************************************************************/
void System_Enable_RC32K(void)
{
    PMU->ANACR |= (1 << 8); 
    while(0 == ((PMU->ANACR) & (1U << 9)));
}

/*********************************************************************************
* Function    : System_Disable_RC32K
* Description : Disable RC32K
* Input       : none  
* Output      : none 
* Author      : CWT                        
**********************************************************************************/
void System_Disable_RC32K(void)
{
    PMU->ANACR &= (~(1 << 8));
}

/*********************************************************************************
* Function    : System_Enable_XTAL
* Description : Enable XTAL, make sure RTC Domain Access is allowed 
* Input       : none 
* Output      : none  
* Author      : CWT                         
**********************************************************************************/
void System_Enable_XTAL(void)
{
    PMU->ANACR = (PMU->ANACR & ~RPMU_ANACR_XTLDRV) | (RPMU_ANACR_XTLDRV_1 | RPMU_ANACR_XTLDRV_0);
    PMU->ANACR |= RPMU_ANACR_XTLEN;
    while(!(PMU->ANACR & RPMU_ANACR_XTLRDY));
    PMU->CR1 |= RTC_CLOCK_XTL;
}

/*********************************************************************************
* Function    : System_Disable_XTAL
* Description : Disable XTAL
* Input       : none 
* Output      : none 
* Author      : CWT                       
**********************************************************************************/
void System_Disable_XTAL(void)
{
    PMU->ANACR &= (~(RPMU_ANACR_XTLEN));   
}

/*********************************************************************************
* Function    : System_Enable_Disable_RTC_Domain_Access
* Description : Enable or Disable  RTC Domain Access.
* Input       : enable or  disable  
* Output      : none   
* Author      : CWT                         
**********************************************************************************/
void System_Enable_Disable_RTC_Domain_Access(FUNC_DISABLE_ENABLE enable_disable)  
{
    if (FUNC_DISABLE == enable_disable)
    {
        SCU->STOPCFG &= (~SCU_STOPCFG_RTC_WE);
    }
    else
    {
        SCU->STOPCFG |= SCU_STOPCFG_RTC_WE;  
        System_Delay(1);     
        RTC->WP = 0xCA53CA53U;  
    }
}

/*********************************************************************************
* Function    : System_Enable_Disable_Reset
* Description : Enable or Disable  System Reset source.
* Input       : none 
* Output      : none 
* Author      : CWT                         
**********************************************************************************/
void System_Enable_Disable_Reset(RESET_ENABLE_SOURCE source, FUNC_DISABLE_ENABLE enable_disable)  
{
    switch(source)
    {
        /* reset source: from bit0 to bit3  */  
        case RESET_ENABLE_SOURCE_LVD:
        case RESET_ENABLE_SOURCE_WDT:
        case RESET_ENABLE_SOURCE_IWDT:
        case RESET_ENABLE_SOURCE_LOCKUP:   

        if (FUNC_DISABLE == enable_disable)
        {
            SCU->RCR &= (~(1U << source));  
        }
        else
        {
            SCU->RCR |= (1U << source);    
        }	
        break; 

        default: break;  
    }
}

/*********************************************************************************
* Function    : System_Reset_MCU
* Description : reset mcu  
* Input       : reset source  
* Output      : none 
* Author      : xwl                          
**********************************************************************************/
void System_Reset_MCU(RESET_SOURCE source)  
{
    switch(source)
    {
        case RESET_SOURCE_EFC:
        {
            SCU->RCR &= (~BIT29);     
            while(1);
        }break;
        
        case RESET_SOURCE_NVIC_RESET:
        {
            NVIC_SystemReset();
            while(1);  
        }break;
        
        case RESET_SOFT_RESET:
        {
            SCU->RCR &= (~BIT30);               
            while(1);
        }break;

        default: break;
    }
}

/*********************************************************************************
* Function    : System_Enter_Standby_Mode   
* Description : try to enter standby mode
* Input       : none         
* Output      : none
* Author      : xwl                         Date : 2021  
**********************************************************************************/
void System_Enter_Standby_Mode(void)
{
    __set_PRIMASK(1);    // disable interrupt 
    SysTick->CTRL = 0;   // disable systick   
    SCB->ICSR = BIT25;   // clear systick pending bit  
    SCU->STOPCFG |= BIT11;  // set PDDS=1  
    
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk)); 
    __WFI();     
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));      
    System_Delay(100);  
    
    printfS("Enter Standby Mode Failed! \n");   // should not go here   
}

/*********************************************************************************
* Function    : System_Clear_Stop_Wakeup
* Description : clear all stop setting and status 
* Input       : none
* Output      : none 
* Author      : CWT                         Date : 2021年
**********************************************************************************/
void System_Clear_Stop_Wakeup(void) 
{
    EXTI->IENR = 0;  
    EXTI->RTENR = 0;  
    EXTI->FTENR = 0;  
    EXTI->SWIER = 0;  
    EXTI->PDR = 0x7FFFFFU;
}

/*********************************************************************************
* Function    : System_Enter_Stop_Mode
* Description : try to enter stop mode
* Input       : STOPEntry: STOPENTRY_WFI or STOPENTRY_WFE             
* Output      : none  
* Author      : CWT                         Date : 2021  
**********************************************************************************/
void System_Enter_Stop_Mode(uint8_t STOPEntry)
{
    /* Set SLEEPDEEP bit of Cortex System Control Register */
    SET_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk)); 
    
    SCU->STOPCFG &= (~BIT11);  // PDDS=0     
    
    System_SysTick_Off();  
    /* Select Stop mode entry */
    if(STOPEntry == STOPENTRY_WFI)
    {
        /* Wait For Interrupt */   
        __WFI();
    }
    else
    {
        __SEV();   
        __WFE();
        __WFE();    /* Wait For Event */  
    }
    
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));  
    
#ifdef HAL_SYSTICK_ENABLED  // To activate macro in ACM32Fxx_HAL.h      
    System_SysTick_Init();
#endif  
}

/*********************************************************************************
* Function    : System_Enter_Sleep_Mode
* Description : try to enter sleep mode
* Input       : SleepEntry: SLEEPENTRY_WFI or SLEEPENTRY_WFE                     
* Output      : none  
* Author      : CWT                         Date : 2021  
**********************************************************************************/
void System_Enter_Sleep_Mode(uint8_t SleepEntry)
{
    /* clear SLEEPDEEP bit of Cortex System Control Register */
    CLEAR_BIT(SCB->SCR, ((uint32_t)SCB_SCR_SLEEPDEEP_Msk));         

    /* Select Stop mode entry */
    if(SleepEntry == SLEEPENTRY_WFI)       
    {
        /* Wait For Interrupt */
        __WFI();
    }
    else
    {
        
        __SEV();      
        __WFE();
        __WFE(); /*Wait For Event */     
    }
        
}


 /*********************************************************************************
* Function    : System_Return_Last_Reset_Reason
* Description : Get System Last Reset Reason
* Input       : none 
* Output      : RESET_REASON
* Author      : CWT                         Date : 2021年
**********************************************************************************/
RESET_REASON System_Return_Last_Reset_Reason(void)
{
    RESET_REASON Reset_Reason_Save;   
    RESET_REASON i = RESET_REASON_POR;    
    
    for(i = RESET_REASON_POR; i >= RESET_REASON_POR12; i--)       
    {
        if ((SCU->RSR) & (1U << i))  
        {
            SCU->RSR |= SCU_RSR_RSTFLAG_CLR; // clear reset reason flags  
            Reset_Reason_Save = i;  
            return i;  
        }
    } 
    
    for(i = RESET_REASON_LOW_VOLTAGE; i <= RESET_REASON_SOFT; i++)    
    {
        if ((SCU->RSR) & (1U << i))    
        {
            SCU->RSR |= SCU_RSR_RSTFLAG_CLR; // clear reset reason flags  
            Reset_Reason_Save = i;  
            return i;  
        }
    }  

    return RESET_REASON_INVALID;  // this should not happen 
}

 /*********************************************************************************
* Function    : System_Set_Buzzer_Divider
* Description : set buzzer divide factor 
* Input       : 
              div: div factor, if div = 80 then output buzzer freq=HCLK/80   
              enable: FUNC_DISABLE and FUNC_ENABLE      
* Output      : none
* Author      : xwl                         Date : 2021?ê
**********************************************************************************/
void System_Set_Buzzer_Divider(uint32_t div, FUNC_DISABLE_ENABLE enable)  
{      
    if (FUNC_ENABLE == enable)
    {
        SCU->CLKOCR = (SCU->CLKOCR & (~(0x1FFFFU << 5) ) ) | (div << 5);      
        SCU->CLKOCR |= BIT23;  
    }
    else
    {
        SCU->CLKOCR &= (~BIT23);   
    }
}

/*********************************************************************************
* Function    : System_USB_PHY_Config
* Description : Configure USB PHY, such as clock select, pll...
* Input       : none  
* Output      : 0: fail, 1:success 
* Author      : xwl                         Date : 2021年
**********************************************************************************/
uint32_t System_USB_PHY_Config(void) 
{
    volatile uint32_t delay_count; 
       
    SCU->PHYCR &= (~BIT2); // exit power down, auto select clock source 
    
    delay_count = SYSTEM_TIMEOUT;  
    while(delay_count--)
    {
        if (SCU->PHYCR & (BIT19)) // clksel_end flag = 1  
        {
            break;  
        }
    }  

    if (delay_count) 
    {
        return HAL_OK;   
    }
    else
    {
        return HAL_TIMEOUT;     
    }
}


