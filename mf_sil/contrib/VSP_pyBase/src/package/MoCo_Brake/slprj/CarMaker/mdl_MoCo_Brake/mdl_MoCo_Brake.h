/*
 * Code generation for system model 'mdl_MoCo_Brake'
 * For more details, see corresponding source file mdl_MoCo_Brake.c
 *
 */

#ifndef RTW_HEADER_mdl_MoCo_Brake_h_
#define RTW_HEADER_mdl_MoCo_Brake_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#include "rtw_modelmap.h"
#ifndef mdl_MoCo_Brake_COMMON_INCLUDES_
# define mdl_MoCo_Brake_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                 /* mdl_MoCo_Brake_COMMON_INCLUDES_ */

#include "mdl_MoCo_Brake_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "model_reference_types.h"
#include "rtGetInf.h"
#include "rt_nonfinite.h"
#include "rt_zcfcn.h"
#include "rtGetNaN.h"

/* Block signals for model 'mdl_MoCo_Brake' */
typedef struct {
  real_T delay_brake_inc_init[2];      /* '<Root>/delay_brake_inc_init' */
  real_T delay_brake_inc[2];           /* '<Root>/delay_brake_inc' */
  real_T brk_dec;                      /* '<Root>/Memory3' */
  real_T Memory;                       /* '<Root>/Memory' */
  real_T Gain31c;                      /* '<Root>/Gain3 [1//c]' */
  real_T Sum2;                         /* '<S2>/Sum2' */
  real_T Integrator2;                  /* '<S3>/Integrator2' */
  real_T Sum2_l;                       /* '<S3>/Sum2' */
  real_T Saturation1;                  /* '<S5>/Saturation1' */
  real_T Subtract1;                    /* '<S5>/Subtract1' */
  real_T Gain2d;                       /* '<Root>/Gain2 [d]' */
  real_T Sum;                          /* '<Root>/Sum' */
  real_T Integrator2_i;                /* '<S7>/Integrator2' */
  real_T Integrator2_j;                /* '<S6>/Integrator2' */
  real_T output[2];                    /* '<S5>/add' */
  real_T Square;                       /* '<S6>/Square' */
  real_T Square1;                      /* '<S6>/Square1' */
  real_T Sum2_g;                       /* '<S6>/Sum2' */
  real_T Square_k;                     /* '<S7>/Square' */
  real_T Square1_f;                    /* '<S7>/Square1' */
  real_T Sum2_gk;                      /* '<S7>/Sum2' */
  real_T accel_deriv;                  /* '<Root>/Filter1' */
  real_T Gain4;                        /* '<Root>/Gain4' */
  real_T Memory1;                      /* '<Root>/Memory1' */
  real_T Memory2;                      /* '<Root>/Memory2' */
  real_T brk_inc_dec_switch;           /* '<Root>/Sign' */
  real_T outp;                         /* '<Root>/Chart1' */
  real_T out_reset;                    /* '<Root>/Chart1' */
} B_mdl_MoCo_Brake_c_T;

/* Block states (default storage) for model 'mdl_MoCo_Brake' */
typedef struct {
  real_T Memory3_PreviousInput;        /* '<Root>/Memory3' */
  real_T Memory4_PreviousInput;        /* '<Root>/Memory4' */
  real_T Memory_PreviousInput;         /* '<Root>/Memory' */
  real_T Memory1_PreviousInput[2];     /* '<S5>/Memory1' */
  real_T Memory_PreviousInput_g;       /* '<S5>/Memory' */
  real_T TimeStampA;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeA;                 /* '<Root>/Derivative' */
  real_T TimeStampB;                   /* '<Root>/Derivative' */
  real_T LastUAtTimeB;                 /* '<Root>/Derivative' */
  real_T PrevY;                        /* '<Root>/Rate Limiter' */
  real_T Memory1_PreviousInput_o;      /* '<Root>/Memory1' */
  real_T Memory2_PreviousInput;        /* '<Root>/Memory2' */
  real_T temp;                         /* '<Root>/Chart1' */
  struct {
    real_T modelTStart;
    real_T TUbufferArea[8192];
  } delay_brake_inc_init_RWORK;        /* '<Root>/delay_brake_inc_init' */

  struct {
    real_T modelTStart;
    real_T TUbufferArea[8192];
  } delay_brake_inc_RWORK;             /* '<Root>/delay_brake_inc' */

  struct {
    void *TUbufferPtrs[4];
  } delay_brake_inc_init_PWORK;        /* '<Root>/delay_brake_inc_init' */

  struct {
    void *TUbufferPtrs[4];
  } delay_brake_inc_PWORK;             /* '<Root>/delay_brake_inc' */

  uint32_T temporalCounter_i1;         /* '<Root>/Chart1' */
  struct {
    int_T Tail[2];
    int_T Head[2];
    int_T Last[2];
    int_T CircularBufSize[2];
  } delay_brake_inc_init_IWORK;        /* '<Root>/delay_brake_inc_init' */

  struct {
    int_T Tail[2];
    int_T Head[2];
    int_T Last[2];
    int_T CircularBufSize[2];
  } delay_brake_inc_IWORK;             /* '<Root>/delay_brake_inc' */

  int_T Integrator2_IWORK;             /* '<S2>/Integrator2' */
  int_T Integrator2_IWORK_d;           /* '<S3>/Integrator2' */
  int_T Integrator1_IWORK;             /* '<S7>/Integrator1' */
  int_T Integrator2_IWORK_n;           /* '<S7>/Integrator2' */
  int_T Integrator1_IWORK_j;           /* '<S6>/Integrator1' */
  int_T Integrator2_IWORK_n0;          /* '<S6>/Integrator2' */
  uint8_T is_active_c3_mdl_MoCo_Brake; /* '<Root>/Chart1' */
  uint8_T is_c3_mdl_MoCo_Brake;        /* '<Root>/Chart1' */
} DW_mdl_MoCo_Brake_f_T;

/* Continuous states for model 'mdl_MoCo_Brake' */
typedef struct {
  real_T Integrator2_CSTATE;           /* '<S2>/Integrator2' */
  real_T Integrator2_CSTATE_f;         /* '<S3>/Integrator2' */
  real_T Integrator1_CSTATE;           /* '<S7>/Integrator1' */
  real_T Integrator2_CSTATE_d;         /* '<S7>/Integrator2' */
  real_T Integrator1_CSTATE_a;         /* '<S6>/Integrator1' */
  real_T Integrator2_CSTATE_o;         /* '<S6>/Integrator2' */
  real_T Filter1_CSTATE;               /* '<Root>/Filter1' */
} X_mdl_MoCo_Brake_n_T;

/* State derivatives for model 'mdl_MoCo_Brake' */
typedef struct {
  real_T Integrator2_CSTATE;           /* '<S2>/Integrator2' */
  real_T Integrator2_CSTATE_f;         /* '<S3>/Integrator2' */
  real_T Integrator1_CSTATE;           /* '<S7>/Integrator1' */
  real_T Integrator2_CSTATE_d;         /* '<S7>/Integrator2' */
  real_T Integrator1_CSTATE_a;         /* '<S6>/Integrator1' */
  real_T Integrator2_CSTATE_o;         /* '<S6>/Integrator2' */
  real_T Filter1_CSTATE;               /* '<Root>/Filter1' */
} XDot_mdl_MoCo_Brake_n_T;

/* State Disabled for model 'mdl_MoCo_Brake' */
typedef struct {
  boolean_T Integrator2_CSTATE;        /* '<S2>/Integrator2' */
  boolean_T Integrator2_CSTATE_f;      /* '<S3>/Integrator2' */
  boolean_T Integrator1_CSTATE;        /* '<S7>/Integrator1' */
  boolean_T Integrator2_CSTATE_d;      /* '<S7>/Integrator2' */
  boolean_T Integrator1_CSTATE_a;      /* '<S6>/Integrator1' */
  boolean_T Integrator2_CSTATE_o;      /* '<S6>/Integrator2' */
  boolean_T Filter1_CSTATE;            /* '<Root>/Filter1' */
} XDis_mdl_MoCo_Brake_n_T;

/* Zero-crossing (trigger) state for model 'mdl_MoCo_Brake' */
typedef struct {
  ZCSigState Integrator2_Reset_ZCE;    /* '<S2>/Integrator2' */
  ZCSigState Integrator2_Reset_ZCE_m;  /* '<S3>/Integrator2' */
  ZCSigState Integrator1_Reset_ZCE;    /* '<S7>/Integrator1' */
  ZCSigState Integrator2_Reset_ZCE_l;  /* '<S7>/Integrator2' */
  ZCSigState Integrator1_Reset_ZCE_o;  /* '<S6>/Integrator1' */
  ZCSigState Integrator2_Reset_ZCE_f;  /* '<S6>/Integrator2' */
} ZCE_mdl_MoCo_Brake_T;

/* Parameters (default storage) */
struct P_mdl_MoCo_Brake_T_ {
  real_T Constant1_Value;              /* Expression: 0
                                        * Referenced by: '<Root>/Constant1'
                                        */
  real_T delay_brake_inc_init_InitOutput;/* Expression: 0
                                          * Referenced by: '<Root>/delay_brake_inc_init'
                                          */
  real_T delay_brake_inc_InitOutput;   /* Expression: 0
                                        * Referenced by: '<Root>/delay_brake_inc'
                                        */
  real_T Memory3_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<Root>/Memory3'
                                        */
  real_T Memory4_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<Root>/Memory4'
                                        */
  real_T Memory_InitialCondition;      /* Expression: 0
                                        * Referenced by: '<Root>/Memory'
                                        */
  real_T Constant_Value;               /* Expression: 0.02
                                        * Referenced by: '<S5>/Constant'
                                        */
  real_T Constant1_Value_b;            /* Expression: 0.1
                                        * Referenced by: '<S5>/Constant1'
                                        */
  real_T Constant2_Value;              /* Expression: 1
                                        * Referenced by: '<S5>/Constant2'
                                        */
  real_T Memory1_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<S5>/Memory1'
                                        */
  real_T Gain_Gain;                    /* Expression: -1
                                        * Referenced by: '<S5>/Gain'
                                        */
  real_T Memory_InitialCondition_g;    /* Expression: 0
                                        * Referenced by: '<S5>/Memory'
                                        */
  real_T Saturation1_UpperSat;         /* Expression: 1
                                        * Referenced by: '<S5>/Saturation1'
                                        */
  real_T Saturation1_LowerSat;         /* Expression: 0
                                        * Referenced by: '<S5>/Saturation1'
                                        */
  real_T Filter1_A;                    /* Computed Parameter: Filter1_A
                                        * Referenced by: '<Root>/Filter1'
                                        */
  real_T Filter1_C;                    /* Computed Parameter: Filter1_C
                                        * Referenced by: '<Root>/Filter1'
                                        */
  real_T Switch1_Threshold;            /* Expression: 0
                                        * Referenced by: '<Root>/Switch1'
                                        */
  real_T RateLimiter_RisingLim;     /* Computed Parameter: RateLimiter_RisingLim
                                     * Referenced by: '<Root>/Rate Limiter'
                                     */
  real_T RateLimiter_FallingLim;   /* Computed Parameter: RateLimiter_FallingLim
                                    * Referenced by: '<Root>/Rate Limiter'
                                    */
  real_T RateLimiter_IC;               /* Expression: 0
                                        * Referenced by: '<Root>/Rate Limiter'
                                        */
  real_T Saturation_UpperSat;          /* Expression: 0
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Saturation_LowerSat;          /* Expression: -inf
                                        * Referenced by: '<Root>/Saturation'
                                        */
  real_T Gain4_Gain;                   /* Expression: -1
                                        * Referenced by: '<Root>/Gain4'
                                        */
  real_T Memory1_InitialCondition_i;   /* Expression: 0
                                        * Referenced by: '<Root>/Memory1'
                                        */
  real_T Memory2_InitialCondition;     /* Expression: 0
                                        * Referenced by: '<Root>/Memory2'
                                        */
  real_T Saturation1_UpperSat_l;       /* Expression: 1
                                        * Referenced by: '<Root>/Saturation1'
                                        */
  real_T Saturation1_LowerSat_a;       /* Expression: 0
                                        * Referenced by: '<Root>/Saturation1'
                                        */
};

/* Real-time Model Data Structure */
struct tag_RTM_mdl_MoCo_Brake_T {
  const char_T **errorStatus;
  RTWSolverInfo *solverInfo;
  const rtTimingBridge *timingBridge;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
    void* dataAddress[75];
    int32_T* vardimsAddress[75];
    RTWLoggingFcnPtr loggingPtrs[75];
  } DataMapInfo;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    time_T stepSize0;
    int_T mdlref_GlobalTID[2];
    SimTimeStep *simTimeStep;
    boolean_T *stopRequestedFlag;
  } Timing;
};

typedef struct {
  B_mdl_MoCo_Brake_c_T rtb;
  DW_mdl_MoCo_Brake_f_T rtdw;
  RT_MODEL_mdl_MoCo_Brake_T rtm;
  ZCE_mdl_MoCo_Brake_T rtzce;
} MdlrefDW_mdl_MoCo_Brake_T;

/* Model block global parameters (default storage) */
extern real_T rtP_Brk_SSM_Decel_Lim;   /* Variable: Brk_SSM_Decel_Lim
                                        * Referenced by: '<Root>/Constant'
                                        */
extern real_T rtP_D_brk;               /* Variable: D_brk
                                        * Referenced by:
                                        *   '<S6>/Gain [f]'
                                        *   '<S7>/Gain [f]'
                                        */
extern real_T rtP_K_brk;               /* Variable: K_brk
                                        * Referenced by:
                                        *   '<S6>/Constant'
                                        *   '<S7>/Constant'
                                        */
extern real_T rtP_K_brk_accel_dec;     /* Variable: K_brk_accel_dec
                                        * Referenced by:
                                        *   '<Root>/Gain1 [c]'
                                        *   '<Root>/Gain3 [1//c]'
                                        *   '<S3>/Gain [c]'
                                        */
extern real_T rtP_K_brk_trq_dec;       /* Variable: K_brk_trq_dec
                                        * Referenced by: '<S2>/Gain [a]'
                                        */
extern real_T rtP_T_brk_accel_dec;     /* Variable: T_brk_accel_dec
                                        * Referenced by:
                                        *   '<Root>/Gain1 [c]'
                                        *   '<Root>/Gain2 [d]'
                                        *   '<Root>/Gain3 [1//c]'
                                        *   '<Root>/Gain4 [d]'
                                        *   '<S3>/Gain [c]'
                                        *   '<S3>/Gain [d]'
                                        */
extern real_T rtP_T_brk_trq_dec;       /* Variable: T_brk_trq_dec
                                        * Referenced by:
                                        *   '<S2>/Gain [a]'
                                        *   '<S2>/Gain [b]'
                                        */
extern real_T rtP_accel_to_trq;        /* Variable: accel_to_trq
                                        * Referenced by: '<Root>/Gain'
                                        */
extern real_T rtP_brake_pres_exist_threshold;/* Variable: brake_pres_exist_threshold
                                              * Referenced by: '<Root>/Chart1'
                                              */
extern real_T rtP_delay_brake_inc;     /* Variable: delay_brake_inc
                                        * Referenced by:
                                        *   '<Root>/Chart1'
                                        *   '<Root>/delay_brake_inc'
                                        */
extern real_T rtP_delay_brake_inc_init;/* Variable: delay_brake_inc_init
                                        * Referenced by:
                                        *   '<Root>/Chart1'
                                        *   '<Root>/delay_brake_inc_init'
                                        */
extern real_T rtP_w0_brk;              /* Variable: w0_brk
                                        * Referenced by:
                                        *   '<S6>/Constant1'
                                        *   '<S6>/Constant2'
                                        *   '<S6>/Gain [f]'
                                        */
extern real_T rtP_w0_brkinit;          /* Variable: w0_brkinit
                                        * Referenced by:
                                        *   '<S7>/Constant1'
                                        *   '<S7>/Constant2'
                                        *   '<S7>/Gain [f]'
                                        */

/* Model reference registration function */
extern void mdl_MoCo_Brake_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RTWSolverInfo *rt_solverInfo, const rtTimingBridge
  *timingBridge, int_T mdlref_TID0, int_T mdlref_TID1, RT_MODEL_mdl_MoCo_Brake_T
  *const mdl_MoCo_Brake_M, B_mdl_MoCo_Brake_c_T *localB, DW_mdl_MoCo_Brake_f_T
  *localDW, X_mdl_MoCo_Brake_n_T *localX, ZCE_mdl_MoCo_Brake_T *localZCE,
  rtwCAPI_ModelMappingInfo *rt_ParentMMI, const char_T *rt_ChildPath, int_T
  rt_ChildMMIIdx, int_T rt_CSTATEIdx);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  mdl_MoCo_Brake_GetCAPIStaticMap(void);
extern void mdl_MoCo_Brake_Init(RT_MODEL_mdl_MoCo_Brake_T * const
  mdl_MoCo_Brake_M, DW_mdl_MoCo_Brake_f_T *localDW, X_mdl_MoCo_Brake_n_T *localX);
extern void mdl_MoCo_Brake_Reset(RT_MODEL_mdl_MoCo_Brake_T * const
  mdl_MoCo_Brake_M, DW_mdl_MoCo_Brake_f_T *localDW, X_mdl_MoCo_Brake_n_T *localX);
extern void mdl_MoCo_Brake_Start(RT_MODEL_mdl_MoCo_Brake_T * const
  mdl_MoCo_Brake_M, DW_mdl_MoCo_Brake_f_T *localDW);
extern void mdl_MoCo_Brake_Deriv(B_mdl_MoCo_Brake_c_T *localB,
  X_mdl_MoCo_Brake_n_T *localX, XDot_mdl_MoCo_Brake_n_T *localXdot);
extern void mdl_MoCo_Brake_Update(RT_MODEL_mdl_MoCo_Brake_T * const
  mdl_MoCo_Brake_M, B_mdl_MoCo_Brake_c_T *localB, DW_mdl_MoCo_Brake_f_T *localDW);
extern void mdl_MoCo_Brake(RT_MODEL_mdl_MoCo_Brake_T * const mdl_MoCo_Brake_M,
  const real_T *rtu_SET_TRQ, const real_T *rtu_SSM_HOLD_REQ, const real_T
  *rtu_SSM_GO, real_T *rty_ACCEL, real_T *rty_Torque_Out, real_T *rty_SSM_Status,
  real_T *rty_SSM_Torque, B_mdl_MoCo_Brake_c_T *localB, DW_mdl_MoCo_Brake_f_T
  *localDW, X_mdl_MoCo_Brake_n_T *localX, ZCE_mdl_MoCo_Brake_T *localZCE);

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'mdl_MoCo_Brake'
 * '<S1>'   : 'mdl_MoCo_Brake/Chart1'
 * '<S2>'   : 'mdl_MoCo_Brake/DEC_PT1_first'
 * '<S3>'   : 'mdl_MoCo_Brake/DEC_PT1_second'
 * '<S4>'   : 'mdl_MoCo_Brake/DEC_PT2'
 * '<S5>'   : 'mdl_MoCo_Brake/DEC_PT2/Blending'
 * '<S6>'   : 'mdl_MoCo_Brake/DEC_PT2/INC_PT2'
 * '<S7>'   : 'mdl_MoCo_Brake/DEC_PT2/INC_PT2_init'
 */
#endif                                 /* RTW_HEADER_mdl_MoCo_Brake_h_ */
