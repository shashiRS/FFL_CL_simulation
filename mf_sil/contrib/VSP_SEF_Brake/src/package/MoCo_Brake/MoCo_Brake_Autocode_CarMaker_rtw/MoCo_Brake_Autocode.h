/*
 * MoCo_Brake_Autocode.h
 *
 * Code generation for model "MoCo_Brake_Autocode".
 *
 * Model version              : 1.52
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Jan 12 13:51:41 2022
 *
 * Target selection: CarMaker.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef RTW_HEADER_MoCo_Brake_Autocode_h_
#define RTW_HEADER_MoCo_Brake_Autocode_h_
#include <stddef.h>
#include <string.h>
#include "rtw_modelmap.h"
#ifndef MoCo_Brake_Autocode_COMMON_INCLUDES_
# define MoCo_Brake_Autocode_COMMON_INCLUDES_
#include <stdlib.h>
#include <Global.h>
#include <TextUtils.h>
#include <DataDict.h>
#include <DirectVarAccess.h>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#endif                                /* MoCo_Brake_Autocode_COMMON_INCLUDES_ */

#include "MoCo_Brake_Autocode_types.h"

/* Shared type includes */
#include "multiword_types.h"
#include "model_reference_types.h"

/* Child system includes */
#include "mdl_MoCo_Brake.h"
#include "rtGetInf.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetBlockIO
# define rtmGetBlockIO(rtm)            ((rtm)->blockIO)
#endif

#ifndef rtmSetBlockIO
# define rtmSetBlockIO(rtm, val)       ((rtm)->blockIO = (val))
#endif

#ifndef rtmGetConstBlockIO
# define rtmGetConstBlockIO(rtm)       ((rtm)->constBlockIO)
#endif

#ifndef rtmSetConstBlockIO
# define rtmSetConstBlockIO(rtm, val)  ((rtm)->constBlockIO = (val))
#endif

#ifndef rtmGetContStateDisabled
# define rtmGetContStateDisabled(rtm)  ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
# define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
# define rtmGetContStates(rtm)         ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
# define rtmSetContStates(rtm, val)    ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
# define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDataMapInfo
# define rtmGetDataMapInfo(rtm)        ((rtm)->DataMapInfo)
#endif

#ifndef rtmSetDataMapInfo
# define rtmSetDataMapInfo(rtm, val)   ((rtm)->DataMapInfo = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
# define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
# define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
# define rtmGetIntgData(rtm)           ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
# define rtmSetIntgData(rtm, val)      ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
# define rtmGetOdeF(rtm)               ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
# define rtmSetOdeF(rtm, val)          ((rtm)->odeF = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
# define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
# define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
# define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
# define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetRootDWork
# define rtmGetRootDWork(rtm)          ((rtm)->dwork)
#endif

#ifndef rtmSetRootDWork
# define rtmSetRootDWork(rtm, val)     ((rtm)->dwork = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
# define rtmGetZCCacheNeedsReset(rtm)  ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
# define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
# define rtmGetdX(rtm)                 ((rtm)->derivs)
#endif

#ifndef rtmSetdX
# define rtmSetdX(rtm, val)            ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetErrorStatusPointer
# define rtmGetErrorStatusPointer(rtm) ((const char_T **)(&((rtm)->errorStatus)))
#endif

#ifndef rtmGetStopRequested
# define rtmGetStopRequested(rtm)      ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
# define rtmSetStopRequested(rtm, val) ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
# define rtmGetStopRequestedPtr(rtm)   (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
# define rtmGetT(rtm)                  (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
# define rtmGetTPtr(rtm)               ((rtm)->Timing.t)
#endif

#define MoCo_Brake_Autocode_M_TYPE     RT_MODEL_MoCo_Brake_Autocode_T

/* Definition required by CarMaker */
#ifndef rtmGetStepSize
# define rtmGetStepSize(rtm)           0.001
#endif

/* Block signals (default storage) */
typedef struct {
  real_T SET_TRQ;                      /* '<Root>/Read CM Dict' */
  real_T brake_torque_defined_positive1;
                                   /* '<Root>/brake_torque_defined_positive1' */
  real_T HOLD_REQ;                     /* '<Root>/Read CM Dict1' */
  real_T GO_REQ;                       /* '<Root>/Read CM Dict2' */
  real_T Model_o1;                     /* '<Root>/Model' */
  real_T Model_o2;                     /* '<Root>/Model' */
  real_T Model_o3;                     /* '<Root>/Model' */
  real_T Model_o4;                     /* '<Root>/Model' */
  real_T brake_torque_defined_positive;
                                    /* '<Root>/brake_torque_defined_positive' */
  real_T Multiply;                     /* '<Root>/Multiply' */
  real_T Multiply1;                    /* '<Root>/Multiply1' */
  real_T Multiply2;                    /* '<Root>/Multiply2' */
  real_T Multiply3;                    /* '<Root>/Multiply3' */
} B_MoCo_Brake_Autocode_T;

/* Block states (default storage) for system '<Root>' */
typedef struct {
  struct {
    void *Entry;
  } ReadCMDict_PWORK;                  /* '<Root>/Read CM Dict' */

  struct {
    void *Entry;
  } ReadCMDict1_PWORK;                 /* '<Root>/Read CM Dict1' */

  struct {
    void *Entry;
  } ReadCMDict2_PWORK;                 /* '<Root>/Read CM Dict2' */

  struct {
    void *Entry;
  } WriteCMDict_PWORK;                 /* '<Root>/Write CM Dict' */

  struct {
    void *Entry;
  } WriteCMDict1_PWORK;                /* '<Root>/Write CM Dict1' */

  struct {
    void *Entry;
  } WriteCMDict2_PWORK;                /* '<Root>/Write CM Dict2' */

  struct {
    void *Entry;
  } WriteCMDict3_PWORK;                /* '<Root>/Write CM Dict3' */

  struct {
    void *Entry;
  } WriteCMDict4_PWORK;                /* '<Root>/Write CM Dict4' */

  struct {
    void *Entry;
  } WriteCMDict5_PWORK;                /* '<Root>/Write CM Dict5' */

  struct {
    void *Entry;
  } WriteCMDict6_PWORK;                /* '<Root>/Write CM Dict6' */

  struct {
    int_T Checked;
  } ReadCMDict_IWORK;                  /* '<Root>/Read CM Dict' */

  struct {
    int_T Checked;
  } ReadCMDict1_IWORK;                 /* '<Root>/Read CM Dict1' */

  struct {
    int_T Checked;
  } ReadCMDict2_IWORK;                 /* '<Root>/Read CM Dict2' */

  struct {
    int_T Checked;
  } WriteCMDict_IWORK;                 /* '<Root>/Write CM Dict' */

  struct {
    int_T Checked;
  } WriteCMDict1_IWORK;                /* '<Root>/Write CM Dict1' */

  struct {
    int_T Checked;
  } WriteCMDict2_IWORK;                /* '<Root>/Write CM Dict2' */

  struct {
    int_T Checked;
  } WriteCMDict3_IWORK;                /* '<Root>/Write CM Dict3' */

  struct {
    int_T Checked;
  } WriteCMDict4_IWORK;                /* '<Root>/Write CM Dict4' */

  struct {
    int_T Checked;
  } WriteCMDict5_IWORK;                /* '<Root>/Write CM Dict5' */

  struct {
    int_T Checked;
  } WriteCMDict6_IWORK;                /* '<Root>/Write CM Dict6' */

  MdlrefDW_mdl_MoCo_Brake_T Model_InstanceData;/* '<Root>/Model' */
} DW_MoCo_Brake_Autocode_T;

/* Continuous states (default storage) */
typedef struct {
  X_mdl_MoCo_Brake_n_T Model_CSTATE;   /* '<Root>/Model' */
} X_MoCo_Brake_Autocode_T;

/* State derivatives (default storage) */
typedef struct {
  XDot_mdl_MoCo_Brake_n_T Model_CSTATE;/* '<Root>/Model' */
} XDot_MoCo_Brake_Autocode_T;

/* State disabled  */
typedef struct {
  XDis_mdl_MoCo_Brake_n_T Model_CSTATE;/* '<Root>/Model' */
} XDis_MoCo_Brake_Autocode_T;

#ifndef ODE1_INTG
#define ODE1_INTG

/* ODE1 Integration Data */
typedef struct {
  real_T *f[1];                        /* derivatives */
} ODE1_IntgData;

#endif

/* Parameters (default storage) */
struct P_MoCo_Brake_Autocode_T_ {
  real_T brake_torque_defined_positive1_Gain;/* Expression: -1
                                              * Referenced by: '<Root>/brake_torque_defined_positive1'
                                              */
  real_T brake_torque_defined_positive_Gain;/* Expression: -1
                                             * Referenced by: '<Root>/brake_torque_defined_positive'
                                             */
};

/* Real-time Model Data Structure */
struct tag_RTM_MoCo_Brake_Autocode_T {
  const char_T *errorStatus;
  RTWSolverInfo *solverInfo;
  rtTimingBridge timingBridge;
  void *blockIO;
  const void *constBlockIO;
  real_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  boolean_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeF[1][7];
  ODE1_IntgData intgData;
  void *dwork;

  /*
   * DataMapInfo:
   * The following substructure contains information regarding
   * structures generated in the model's C API.
   */
  struct {
    rtwCAPI_ModelMappingInfo mmi;
    void* dataAddress[32];
    int32_T* vardimsAddress[32];
    RTWLoggingFcnPtr loggingPtrs[32];
    rtwCAPI_ModelMappingInfo* childMMI[1];
  } DataMapInfo;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

/* Block parameters (default storage) */
extern P_MoCo_Brake_Autocode_T MoCo_Brake_Autocode_P;

/* External data declarations for dependent source files */
extern const char *RT_MEMORY_ALLOCATION_ERROR;

/* Model block global parameters (default storage) */
extern real_T rtP_Brk_SSM_Decel_Lim;   /* Variable: Brk_SSM_Decel_Lim
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_D_brk;               /* Variable: D_brk
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_K_brk;               /* Variable: K_brk
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_K_brk_accel_dec;     /* Variable: K_brk_accel_dec
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_K_brk_trq_dec;       /* Variable: K_brk_trq_dec
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_T_brk_accel_dec;     /* Variable: T_brk_accel_dec
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_T_brk_trq_dec;       /* Variable: T_brk_trq_dec
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_Trq_Distrib_FL;      /* Variable: Trq_Distrib_FL
                                        * Referenced by: '<Root>/Constant'
                                        */
extern real_T rtP_Trq_Distrib_FR;      /* Variable: Trq_Distrib_FR
                                        * Referenced by: '<Root>/Constant1'
                                        */
extern real_T rtP_Trq_Distrib_RL;      /* Variable: Trq_Distrib_RL
                                        * Referenced by: '<Root>/Constant2'
                                        */
extern real_T rtP_Trq_Distrib_RR;      /* Variable: Trq_Distrib_RR
                                        * Referenced by: '<Root>/Constant3'
                                        */
extern real_T rtP_accel_to_trq;        /* Variable: accel_to_trq
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_brake_pres_exist_threshold;/* Variable: brake_pres_exist_threshold
                                              * Referenced by: '<Root>/Model'
                                              */
extern real_T rtP_delay_brake_inc;     /* Variable: delay_brake_inc
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_delay_brake_inc_init;/* Variable: delay_brake_inc_init
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_w0_brk;              /* Variable: w0_brk
                                        * Referenced by: '<Root>/Model'
                                        */
extern real_T rtP_w0_brkinit;          /* Variable: w0_brkinit
                                        * Referenced by: '<Root>/Model'
                                        */

/* Model entry point functions */
struct tInfos;
extern RT_MODEL_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode(struct tInfos *inf);
extern void MoCo_Brake_Autocode_initialize(RT_MODEL_MoCo_Brake_Autocode_T *const
  MoCo_Brake_Autocode_M);
extern void MoCo_Brake_Autocode_step(RT_MODEL_MoCo_Brake_Autocode_T *const
  MoCo_Brake_Autocode_M);
extern void MoCo_Brake_Autocode_terminate(RT_MODEL_MoCo_Brake_Autocode_T
  * MoCo_Brake_Autocode_M);

/* Function to get C API Model Mapping Static Info */
extern const rtwCAPI_ModelMappingStaticInfo*
  MoCo_Brake_Autocode_GetCAPIStaticMap(void);

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
 * '<Root>' : 'MoCo_Brake_Autocode'
 */
#endif                                 /* RTW_HEADER_MoCo_Brake_Autocode_h_ */
