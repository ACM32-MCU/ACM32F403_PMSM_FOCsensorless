#ifndef RTW_HEADER_FOC_CURRENT_h_
#define RTW_HEADER_FOC_CURRENT_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef FOC_CURRENT_COMMON_INCLUDES_
#define FOC_CURRENT_COMMON_INCLUDES_
#include "rtwtypes.h"
#endif

#include "FOC_CURRENT_types.h"

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

typedef struct {
    real32_T Integrator_DSTATE;
    real32_T Integrator_DSTATE_h;
} DW_CTRL_CUR_FOC_CURRENT_T;

typedef struct {
    real32_T UnitDelay_DSTATE;
    real32_T DiscreteTimeIntegrator1_DSTATE;
    real32_T DiscreteTimeIntegrator_DSTATE;
    real32_T Integrator_DSTATE;
    real32_T Integrator1_DSTATE;
    real32_T Integrator_DSTATE_l;
    DW_CTRL_CUR_FOC_CURRENT_T CTRL_CUR;
} DW_FOC_CURRENT_T;

typedef struct {
    real32_T ISensA;
    real32_T ISensB;
    real32_T ISensC;
    real32_T VBus;
} ExtU_FOC_CURRENT_T;

typedef struct {
    real32_T tAout;
    real32_T tBout;
    real32_T tCout;
} ExtY_FOC_CURRENT_T;

typedef struct MotorParam_tag {
    real32_T Flux;
    real32_T Gamma;
    real32_T LADRC_Omega_c;
    real32_T LADRC_b0;
    real32_T Ls;
    real32_T MaxVoltage;
    real32_T MaxVoltage1;
    real32_T PolePairs;
    real32_T Rs;
    real32_T pll_omega;
    real32_T pll_xi;
} MotorParam_type;

typedef struct ObserverParam_tag {
    real32_T ObserverTheta;
    real32_T RefIq;
    real32_T RefRPM;
} ObserverParam_type;

struct tag_RTM_FOC_CURRENT_T {
    const char_T * volatile errorStatus;
};

extern DW_FOC_CURRENT_T FOC_CURRENT_DW;
extern ExtU_FOC_CURRENT_T FOC_CURRENT_U;
extern ExtY_FOC_CURRENT_T FOC_CURRENT_Y;
extern void FOC_CURRENT_initialize(void);
extern void FOC_CURRENT_step(void);
extern void FOC_CURRENT_terminate(void);
extern MotorParam_type MotorParam;
extern ObserverParam_type ObserverParam;
extern RT_MODEL_FOC_CURRENT_T *const FOC_CURRENT_M;

#endif

