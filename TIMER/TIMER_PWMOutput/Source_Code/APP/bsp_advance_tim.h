// UTF-8
#ifndef __ADVANCE_TIM_H
#define	__ADVANCE_TIM_H

#include "ACM32Fxx_HAL.h"

/* 死区发生器采样时钟为180MHz */
#define DEADTIME   0x5A  //500ns

/* 定义定时器时钟频率(CK_CNT) ，90MHz */
#define ADVANCED_TIM_FREQ     90000000U

/* 定义定时器周期 */
#define ADVANCED_TIM_PERIOD        4500    

/* PWM最大占空比，96% */
#define MAX_PWM_DUTY               4200 
  
/* 定义高级定时器重复计数次数 */
#define ADVANCED_TIM_REPETITIONCOUNTER      0

/* 用于计算占空比 */
#define PWM_DUTY                   2250


void Init_Gpio_TIM1_PWM(void);
void StartPWM(void);
void StopPWM(void);


#endif /* __ADVANCE_TIM_H */


