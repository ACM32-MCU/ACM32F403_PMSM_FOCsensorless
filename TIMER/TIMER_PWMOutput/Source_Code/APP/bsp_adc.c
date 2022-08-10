//UTF-8
#include "bsp_adc.h"
#include "arm_math.h"
#include "math.h"
#include "stdlib.h"
#include "FOC_CURRENT.h"
#include "bsp_advance_tim.h"

MotorParam_type MotorParam = {

    0.0109F,       //磁链值

    600000.0F,     //观测器增益

    100.0F,        //速度环Ki

    8000.0F,       //速度环Kp

	  0.000195185F,  //电机电感

	  10.0F,          //电机最大调制电压

    -10.0F,        //最小调制电压

   	5.0F,            //电机极对数

	  0.166F,          //电机电阻值

    5000.0F,        //PLL的Kp

    10.0F           //PLL的Ki
};

volatile uint32_t flag = 0;
stMcInfo mc_info;	
static void ParaInit(void);		// parameters initialization.

extern uint32_t ADC_VrefP;
ADC_HandleTypeDef ADC_Handle;
uint32_t ADC1ConvertedValue[5];

uint16_t phaseA, phaseB, phaseC, total;

/* 根据相位设置对不同的ADC通道采样 */
void SetADCSequenceChannel(uint32_t sequence, uint32_t channel)
{
    if((sequence >= 1)&&(sequence <= 5)) 
        MODIFY_REG(ADC_Handle.Instance->SQR1,(ADC_CH_MASK << (5*sequence)),(channel << sequence));
    else if((sequence >= 6)&&(sequence <= 11))
        MODIFY_REG(ADC_Handle.Instance->SQR2,(ADC_CH_MASK << (5*sequence-6)),(channel << (5*sequence)));
    else if((sequence >= 12)&&(sequence <= 16))
        MODIFY_REG(ADC_Handle.Instance->SQR3,(ADC_CH_MASK << (5*(sequence-12))),(channel << (5*(sequence-12))));
}


void DMA_IRQHandler(void)
{
    /* Transfer complete interrupt(CH0) */
    if (DMA->INT_TC_STATUS & 1)
    {
        DMA->INT_TC_CLR |= 1;
   
        //TODO:DMA传输完成，根据实际应用处理ADC1ConvertedValue[]中的数据
        phaseA = (ADC1ConvertedValue[0]&0xFFF) ;
        phaseC = (ADC1ConvertedValue[1]&0xFFF) ;
        total  = (ADC1ConvertedValue[2]&0xFFF) ;

#if 1
    mc_info.isens_a = phaseA * 0.0155f - 33.0f ;
    mc_info.isens_c = phaseC * 0.0155f - 33.0f ;
    mc_info.isens_b = -( mc_info.isens_a + mc_info.isens_c);
    mc_info.vbus   = total * 0.0156F;
			
    mc_info.cmd=1;
			
   switch( mc_info.mc_state)
		{
			case MC_RDY:
			case MC_STOP:
				if(mc_info.cmd == START_CMD)
				{
					FOC_CURRENT_initialize();
					ParaInit();
					mc_info.mc_state = MC_START;
				}
				else
				{
				
				}
				break;

			case MC_START:			
				mc_info.mc_state = MC_RUN;
			//	StartPWM();
				break;

			case MC_RUN:
				FOC_CURRENT_U.ISensA = mc_info.isens_a;
				FOC_CURRENT_U.ISensB = mc_info.isens_b;
				FOC_CURRENT_U.ISensC = mc_info.isens_c;
				FOC_CURRENT_U.VBus   = mc_info.vbus;
			//	FOC_CURRENT_U.Iqr= mc_info.refIq;
			  ObserverParam.RefRPM=mc_info.refRPM;
			
				FOC_CURRENT_step();
				TIM1->CCR1 = (FOC_CURRENT_Y.tAout > MAX_PWM_DUTY) ? MAX_PWM_DUTY : FOC_CURRENT_Y.tAout;
				TIM1->CCR2 = (FOC_CURRENT_Y.tBout > MAX_PWM_DUTY) ? MAX_PWM_DUTY : FOC_CURRENT_Y.tBout;
				TIM1->CCR3 = (FOC_CURRENT_Y.tCout > MAX_PWM_DUTY) ? MAX_PWM_DUTY : FOC_CURRENT_Y.tCout;

				if( mc_info.cmd == STOP_CMD)
				{
					mc_info.mc_state = MC_STOP;
				
				
				}
				if( mc_info.mc_err != NONE_ERR)
				{
					mc_info.mc_state = MC_ERR;
				}
				break;

			case MC_ERR:
			//	StopPWM();
				break;

			default:
				break;
		}
			
			if( mc_info.vbus < 20.0f)
		{
			mc_info.mc_err = LV_ERR;
		//	StopPWM();
		}
		else if( mc_info.vbus > 55.0f)
		{
		//	StopPWM();
			if( mc_info.over_vol_count++ > 5)
			{
				mc_info.mc_err = OV_ERR;
				mc_info.err_state_vbus = mc_info.vbus;
			}
		}
				
#endif
    }
}

void ParaInit(void)
{
	mc_info.refIq = 0.0f;
	mc_info	.refRPM = 0.0f;
	mc_info.mc_state = MC_RDY;
	mc_info.mc_err = NONE_ERR;
}


/************************************************************************
 * function   : ADC_GetVrefP
 * Description: ADC Get The VrefP Value. 
 * input      : Pre_Channel:Can be ADC_CHANNEL_2 ADC_CHANNEL_8 ADC_CHANNEL_10
 ************************************************************************/ 
uint32_t ADC_GetVrefP(uint8_t Pre_Channel)
{
    ADC_ChannelConfTypeDef ADC_ChannelConf;

    uint32_t TrimValue_3v, AdcValue_VrefP[2], VrefP,temp;
    
    ADC_Handle.Init.ClockDiv = ADC_CLOCK_DIV8;
    ADC_Handle.Init.ConConvMode = ADC_CONCONVMODE_DISABLE;
    ADC_Handle.Init.JChannelMode = ADC_JCHANNELMODE_DISABLE;
    ADC_Handle.Init.DiffMode = ADC_DIFFMODE_DISABLE;
    ADC_Handle.Init.DMAMode = ADC_DMAMODE_DISABLE;
    ADC_Handle.Init.OverMode = ADC_OVERMODE_DISABLE;
    ADC_Handle.Init.OverSampMode = ADC_OVERSAMPMODE_DISABLE;
    ADC_Handle.Init.AnalogWDGEn = ADC_ANALOGWDGEN_DISABLE;
    ADC_Handle.Init.ExTrigMode.ExTrigSel = ADC_SOFTWARE_START;
    ADC_Handle.Init.ChannelEn = ADC_CHANNEL_VBGR_EN;

    ADC_Handle.Instance = ADC;
        
    HAL_ADC_Init(&ADC_Handle);

    /* The total adc regular channels number */
    ADC_Handle.ChannelNum = 2;
    
     /* Add adc channels */
    ADC_ChannelConf.Channel = Pre_Channel;
    ADC_ChannelConf.RjMode = 0;
    ADC_ChannelConf.Sq = ADC_SEQUENCE_SQ1;
    ADC_ChannelConf.Smp = ADC_SMP_CLOCK_320;    
    HAL_ADC_ConfigChannel(&ADC_Handle,&ADC_ChannelConf);
    
    /* Add adc channels */
    ADC_ChannelConf.Channel = ADC_CHANNEL_VBGR;
    ADC_ChannelConf.RjMode = 0;
    ADC_ChannelConf.Sq = ADC_SEQUENCE_SQ2;
    ADC_ChannelConf.Smp = ADC_SMP_CLOCK_320;    
    HAL_ADC_ConfigChannel(&ADC_Handle,&ADC_ChannelConf);
    
    HAL_ADC_Polling(&ADC_Handle, AdcValue_VrefP, ADC_Handle.ChannelNum, 0);
    
    printfS("The adc convert result :  0x%08x \r\n", AdcValue_VrefP[1]);
    
    TrimValue_3v = *(volatile uint32_t*)(0x00080240); //Read the 1.2v trim value in 3.0v vrefp.

    printfS("The adc 1.2v trim value is : 0x%08x \r\n", TrimValue_3v);

    if(((~TrimValue_3v&0xFFFF0000)>>16) == (TrimValue_3v&0x0000FFFF))
    {
        temp = TrimValue_3v & 0xFFF;
        
        VrefP = (uint32_t)(temp * 3000 / (AdcValue_VrefP[1] & 0xFFF));

        printfS("The adc VrefP value is : %dmV \r\n", VrefP);

        return VrefP;
    }
    else
        return 0;
}

void ADC_Motor(void)
{
    ADC_ChannelConfTypeDef ADC_ChannelConf;
    DMA_HandleTypeDef  Dma_Adc_Handle;
    
    ADC_Handle.Instance = ADC;
    /* ADC_CLK = PCLK/2, 90/2=45MHz */
    ADC_Handle.Init.ClockDiv = ADC_CLOCK_DIV2;  
    //ADC_Handle.Init.ClockDiv = ADC_CLOCK_DIV8; 
    /* 连续转换模式禁止 */  
    ADC_Handle.Init.ConConvMode = ADC_CONCONVMODE_DISABLE;
    /* 不支持注入通道 */
    ADC_Handle.Init.JChannelMode = ADC_JCHANNELMODE_DISABLE;
    /* 不使用差分模式 */
    ADC_Handle.Init.DiffMode = ADC_DIFFMODE_DISABLE;
    /* 使能转换结果的DMA传输 */
    ADC_Handle.Init.DMAMode = ADC_DMAMODE_ENABLE;
    /* 数据溢出丢弃 */
    ADC_Handle.Init.OverMode = ADC_OVERMODE_DISABLE;
    /* 不使用过采样 */
    ADC_Handle.Init.OverSampMode = ADC_OVERSAMPMODE_DISABLE;
    /* 不开启模拟看门狗 */
    ADC_Handle.Init.AnalogWDGEn = ADC_ANALOGWDGEN_DISABLE;
    /* TIM1更新事件触发ADC */
    ADC_Handle.Init.ExTrigMode.ExTrigSel = ADC_EXTERNAL_TIG1;
    /* 使能ADC通道，下述引脚默认是模拟输入功能，因此不进行IO配置。 
       PHASE_A:PA0 -- ADC_IN6, 
       PHASE_B:PC1 -- ADC_IN5,
       PHASE_C:PC0 -- ADC_IN13,  
       VBUS   :PA1 -- ADC_VBAT,建议换个引脚，因为改ADC内部有分压
     */
    ADC_Handle.Init.ChannelEn = ADC_CHANNEL_6_EN | ADC_CHANNEL_5_EN 
                               | ADC_CHANNEL_13_EN | ADC_CHANNEL_10_EN;//ADC_CHANNEL_VBAT_EN;

    ADC_Handle.AdcResults = &ADC1ConvertedValue[0];                           
     
    HAL_ADC_Init(&ADC_Handle);

    /* 采样2相电流加母线电流 */
    ADC_Handle.ChannelNum = 3;
    
    /* 采样PHASE_A(CH6)电流 */
    ADC_ChannelConf.Channel = ADC_CHANNEL_6;
    /* 规则模式 */
    ADC_ChannelConf.RjMode = 0;
    /* 转换序号为1 */
    ADC_ChannelConf.Sq = ADC_SEQUENCE_SQ1;
    /* 采样时间(时钟个数) */
    /* ADC转换时间 = (20+17)/45Mhz = 822ns */
    ADC_ChannelConf.Smp = ADC_SMP_CLOCK_10;    
    //ADC_ChannelConf.Smp = ADC_SMP_CLOCK_320; 
    HAL_ADC_ConfigChannel(&ADC_Handle,&ADC_ChannelConf);
    
    /* 采样PHASE_C(CH13)电流 */
    ADC_ChannelConf.Channel = ADC_CHANNEL_13;
    /* 规则模式 */
    ADC_ChannelConf.RjMode = 0;
    /* 转换序号为2 */
    ADC_ChannelConf.Sq = ADC_SEQUENCE_SQ2;
    /* 采样时间(时钟个数) */
    ADC_ChannelConf.Smp = ADC_SMP_CLOCK_10;    
    //ADC_ChannelConf.Smp = ADC_SMP_CLOCK_320;  
    HAL_ADC_ConfigChannel(&ADC_Handle,&ADC_ChannelConf);
    
    /* 采样母线电流 */
    ADC_ChannelConf.Channel = ADC_CHANNEL_10;
    /* 规则模式 */
    ADC_ChannelConf.RjMode = 0;
    /* 转换序号为3 */
    ADC_ChannelConf.Sq = ADC_SEQUENCE_SQ3;
    /* 采样时间(时钟个数) */
    ADC_ChannelConf.Smp = ADC_SMP_CLOCK_10;    
    //ADC_ChannelConf.Smp = ADC_SMP_CLOCK_320;  
    HAL_ADC_ConfigChannel(&ADC_Handle,&ADC_ChannelConf);

    /* ADC DMA 初始化 */
    /* 选择DMA通道 */
    Dma_Adc_Handle.Instance = DMA_Channel0;
    /* 选择DMA请求源为ADC */
    Dma_Adc_Handle.Init.Request_ID = REQ0_ADC;
    /* 使能循环模式 */
    Dma_Adc_Handle.Init.Mode = DMA_CIRCULAR;  
    /* 数据流方向：外设->存储 */      
    Dma_Adc_Handle.Init.Data_Flow = DMA_DATA_FLOW_P2M;
    /* 源地址不递增，目标地址递增 */
    Dma_Adc_Handle.Init.Source_Inc       = DMA_SOURCE_ADDR_INCREASE_DISABLE;
    Dma_Adc_Handle.Init.Desination_Inc   = DMA_DST_ADDR_INCREASE_ENABLE;
    /* 源位宽和目标位宽，字（32bits） */
    Dma_Adc_Handle.Init.Source_Width     = DMA_SRC_WIDTH_WORD;
    Dma_Adc_Handle.Init.Desination_Width = DMA_DST_WIDTH_WORD;

    /* DMA完成中断和错误中断回调函数 */
    /*-----------------------------------------------------------------------------------*/
    /* Note:If user dons not apply interrupt, Set DMA_ITC_CallbackbDMA_IE_Callback NULL */
    /*-----------------------------------------------------------------------------------*/
    Dma_Adc_Handle.DMA_ITC_Callback = NULL;
    Dma_Adc_Handle.DMA_IE_Callback  = NULL;

    HAL_DMA_Init(&Dma_Adc_Handle);

    ADC_Handle.DMA_Handle = &Dma_Adc_Handle;

    
    HAL_ADC_Start_DMA(&ADC_Handle, ADC_Handle.AdcResults, ADC_Handle.ChannelNum);

    /* DMA中断优先级配置为0 */
    NVIC_SetPriority(DMA_IRQn, 0);
}


//===========================================================================
// No more.
//===========================================================================
