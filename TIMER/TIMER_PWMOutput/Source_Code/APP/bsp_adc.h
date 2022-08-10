#ifndef __BSP_ADC__H_H
#define	__BSP_ADC__H_H
// UTF-8

#include "ACM32Fxx_HAL.h"

uint32_t ADC_GetVrefP(uint8_t Pre_Channel);
#define PI_2		6.2832f


typedef enum
{
	NONE_ERR,
	LV_ERR,
	OV_ERR,
	OC_ERR
} enMcErr;

typedef enum
{
	MC_RDY,
	MC_START,
	MC_RUN,
	MC_STOP,
	MC_ERR
} enMcState;

typedef enum
{
	START_CMD = 1,
	STOP_CMD = 0
} enMcCMD;

typedef struct
{
	enMcCMD cmd;
	enMcState mc_state;
	enMcErr	mc_err;
	float refFreq;
	float refIq;
	float refId;
	float refRPM;
	float refPos;

	float isens_a;
	float isens_b;
	float isens_c;
	float vbus;
	float encoder_theta;
	uint16_t encoder_dir;
	int16_t encoder_count;
	
	float run_realtime;
	uint8_t discharge_on;
	uint8_t err_code;
	uint8_t over_vol_count;
	
	float err_state_vbus;
} stMcInfo;

extern void BspCordicInit(void);
extern void BspInit(void);

void ADC_Motor(void);

#endif
