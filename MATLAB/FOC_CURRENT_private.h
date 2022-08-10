#ifndef RTW_HEADER_FOC_CURRENT_private_h_
#define RTW_HEADER_FOC_CURRENT_private_h_
#include "rtwtypes.h"
#include "FOC_CURRENT.h"

extern void FOC_CURRENT_CTRL_CUR(real32_T rtu_RefId, real32_T rtu_ISensD,
    real32_T rtu_RefIq, real32_T rtu_ISensQ, real32_T *rty_PIOutputVd, real32_T *
    rty_PIOutputVq, DW_CTRL_CUR_FOC_CURRENT_T *localDW);
extern void FOC_CURRENT_InvPark(real32_T rtu_d, real32_T rtu_q, const real32_T
    rtu_SinCos[2], real32_T *rty_alpha, real32_T *rty_beta);
extern void FOC_CURRENT_Park(real32_T rtu_Alpha, real32_T rtu_Beta, const
    real32_T rtu_SinCos[2], real32_T *rty_D, real32_T *rty_Q);
extern void FOC_CURRENT_SVPWM2(real32_T rtu_Valpha, real32_T rtu_Vbeta, real32_T
    rtu_v_bus, real32_T rty_tABC[3]);

#endif

