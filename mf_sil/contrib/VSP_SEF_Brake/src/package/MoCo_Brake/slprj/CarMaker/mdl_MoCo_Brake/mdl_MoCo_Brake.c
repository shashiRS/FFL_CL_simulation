/*
 * Code generation for system model 'mdl_MoCo_Brake'
 *
 * Model                      : mdl_MoCo_Brake
 * Model version              : 1.3
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Jan 12 13:51:23 2022
 *
 * Note that the functions contained in this file are part of a Simulink
 * model, and are not self-contained algorithms.
 */

#include "mdl_MoCo_Brake_capi.h"
#include <MatSupp.h>
#include "mdl_MoCo_Brake.h"
#include "mdl_MoCo_Brake_private.h"
#include "rt_TDelayInterpolate.h"

/* Named constants for Chart: '<Root>/Chart1' */
#define mdl_MoCo_Brake_IN_Inc_Trans    ((uint8_T)3U)
#define mdl_MoCo_Brake_IN_Inc_Trans1   ((uint8_T)4U)
#define mdl_MoCo_Brake_IN_Inc_Trans_init ((uint8_T)1U)
#define mdl_MoCo_Brake_IN_Inc_init     ((uint8_T)2U)
#define mdl_MoCo_Brake_IN_NO_ACTIVE_CHILD ((uint8_T)0U)
#define mdl_MoCo_Brake_IN_dec          ((uint8_T)5U)

P_mdl_MoCo_Brake_T mdl_MoCo_Brake_P = {
  /* Expression: 0
   * Referenced by: '<Root>/Constant1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/delay_brake_inc_init'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/delay_brake_inc'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Memory3'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Memory4'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Memory'
   */
  0.0,

  /* Expression: 0.02
   * Referenced by: '<S5>/Constant'
   */
  0.02,

  /* Expression: 0.1
   * Referenced by: '<S5>/Constant1'
   */
  0.1,

  /* Expression: 1
   * Referenced by: '<S5>/Constant2'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S5>/Memory1'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<S5>/Gain'
   */
  -1.0,

  /* Expression: 0
   * Referenced by: '<S5>/Memory'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<S5>/Saturation1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<S5>/Saturation1'
   */
  0.0,

  /* Computed Parameter: Filter1_A
   * Referenced by: '<Root>/Filter1'
   */
  -20.0,

  /* Computed Parameter: Filter1_C
   * Referenced by: '<Root>/Filter1'
   */
  20.0,

  /* Expression: 0
   * Referenced by: '<Root>/Switch1'
   */
  0.0,

  /* Computed Parameter: RateLimiter_RisingLim
   * Referenced by: '<Root>/Rate Limiter'
   */
  0.008,

  /* Computed Parameter: RateLimiter_FallingLim
   * Referenced by: '<Root>/Rate Limiter'
   */
  -0.008,

  /* Expression: 0
   * Referenced by: '<Root>/Rate Limiter'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Saturation'
   */
  0.0,

  /* Expression: -inf
   * Referenced by: '<Root>/Saturation'
   */
  0.0,

  /* Expression: -1
   * Referenced by: '<Root>/Gain4'
   */
  -1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Memory1'
   */
  0.0,

  /* Expression: 0
   * Referenced by: '<Root>/Memory2'
   */
  0.0,

  /* Expression: 1
   * Referenced by: '<Root>/Saturation1'
   */
  1.0,

  /* Expression: 0
   * Referenced by: '<Root>/Saturation1'
   */
  0.0
};

/* System initialize for referenced model: 'mdl_MoCo_Brake' */
void mdl_MoCo_Brake_Init(RT_MODEL_mdl_MoCo_Brake_T * const mdl_MoCo_Brake_M,
  DW_mdl_MoCo_Brake_f_T *localDW, X_mdl_MoCo_Brake_n_T *localX)
{
  /* InitializeConditions for Memory: '<Root>/Memory3' */
  localDW->Memory3_PreviousInput = mdl_MoCo_Brake_P.Memory3_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory4' */
  localDW->Memory4_PreviousInput = mdl_MoCo_Brake_P.Memory4_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory' */
  localDW->Memory_PreviousInput = mdl_MoCo_Brake_P.Memory_InitialCondition;

  /* InitializeConditions for Integrator: '<S2>/Integrator2' incorporates:
   *  Integrator: '<S3>/Integrator2'
   */
  if (rtmIsFirstInitCond(mdl_MoCo_Brake_M)) {
    localX->Integrator2_CSTATE = 0.0;
    localX->Integrator2_CSTATE_f = 0.0;
  }

  localDW->Integrator2_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S2>/Integrator2' */

  /* InitializeConditions for Integrator: '<S3>/Integrator2' */
  localDW->Integrator2_IWORK_d = 1;

  /* InitializeConditions for Memory: '<S5>/Memory1' */
  localDW->Memory1_PreviousInput[0] = mdl_MoCo_Brake_P.Memory1_InitialCondition;
  localDW->Memory1_PreviousInput[1] = mdl_MoCo_Brake_P.Memory1_InitialCondition;

  /* InitializeConditions for Memory: '<S5>/Memory' */
  localDW->Memory_PreviousInput_g = mdl_MoCo_Brake_P.Memory_InitialCondition_g;

  /* InitializeConditions for Integrator: '<S7>/Integrator1' incorporates:
   *  Integrator: '<S7>/Integrator2'
   */
  if (rtmIsFirstInitCond(mdl_MoCo_Brake_M)) {
    localX->Integrator1_CSTATE = 0.0;
    localX->Integrator2_CSTATE_d = 0.0;
  }

  localDW->Integrator1_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S7>/Integrator1' */

  /* InitializeConditions for Integrator: '<S7>/Integrator2' */
  localDW->Integrator2_IWORK_n = 1;

  /* InitializeConditions for Integrator: '<S6>/Integrator1' incorporates:
   *  Integrator: '<S6>/Integrator2'
   */
  if (rtmIsFirstInitCond(mdl_MoCo_Brake_M)) {
    localX->Integrator1_CSTATE_a = 0.0;
    localX->Integrator2_CSTATE_o = 0.0;
  }

  localDW->Integrator1_IWORK_j = 1;

  /* End of InitializeConditions for Integrator: '<S6>/Integrator1' */

  /* InitializeConditions for Integrator: '<S6>/Integrator2' */
  localDW->Integrator2_IWORK_n0 = 1;

  /* InitializeConditions for TransferFcn: '<Root>/Filter1' */
  localX->Filter1_CSTATE = 0.0;

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  localDW->TimeStampA = (rtInf);
  localDW->TimeStampB = (rtInf);

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  localDW->PrevY = mdl_MoCo_Brake_P.RateLimiter_IC;

  /* InitializeConditions for Memory: '<Root>/Memory1' */
  localDW->Memory1_PreviousInput_o = mdl_MoCo_Brake_P.Memory1_InitialCondition_i;

  /* InitializeConditions for Memory: '<Root>/Memory2' */
  localDW->Memory2_PreviousInput = mdl_MoCo_Brake_P.Memory2_InitialCondition;

  /* SystemInitialize for Chart: '<Root>/Chart1' */
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c3_mdl_MoCo_Brake = 0U;
  localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_NO_ACTIVE_CHILD;
}

/* System reset for referenced model: 'mdl_MoCo_Brake' */
void mdl_MoCo_Brake_Reset(RT_MODEL_mdl_MoCo_Brake_T * const mdl_MoCo_Brake_M,
  DW_mdl_MoCo_Brake_f_T *localDW, X_mdl_MoCo_Brake_n_T *localX)
{
  /* InitializeConditions for Memory: '<Root>/Memory3' */
  localDW->Memory3_PreviousInput = mdl_MoCo_Brake_P.Memory3_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory4' */
  localDW->Memory4_PreviousInput = mdl_MoCo_Brake_P.Memory4_InitialCondition;

  /* InitializeConditions for Memory: '<Root>/Memory' */
  localDW->Memory_PreviousInput = mdl_MoCo_Brake_P.Memory_InitialCondition;

  /* InitializeConditions for Integrator: '<S2>/Integrator2' incorporates:
   *  Integrator: '<S3>/Integrator2'
   */
  if (rtmIsFirstInitCond(mdl_MoCo_Brake_M)) {
    localX->Integrator2_CSTATE = 0.0;
    localX->Integrator2_CSTATE_f = 0.0;
  }

  localDW->Integrator2_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S2>/Integrator2' */

  /* InitializeConditions for Integrator: '<S3>/Integrator2' */
  localDW->Integrator2_IWORK_d = 1;

  /* InitializeConditions for Memory: '<S5>/Memory1' */
  localDW->Memory1_PreviousInput[0] = mdl_MoCo_Brake_P.Memory1_InitialCondition;
  localDW->Memory1_PreviousInput[1] = mdl_MoCo_Brake_P.Memory1_InitialCondition;

  /* InitializeConditions for Memory: '<S5>/Memory' */
  localDW->Memory_PreviousInput_g = mdl_MoCo_Brake_P.Memory_InitialCondition_g;

  /* InitializeConditions for Integrator: '<S7>/Integrator1' incorporates:
   *  Integrator: '<S7>/Integrator2'
   */
  if (rtmIsFirstInitCond(mdl_MoCo_Brake_M)) {
    localX->Integrator1_CSTATE = 0.0;
    localX->Integrator2_CSTATE_d = 0.0;
  }

  localDW->Integrator1_IWORK = 1;

  /* End of InitializeConditions for Integrator: '<S7>/Integrator1' */

  /* InitializeConditions for Integrator: '<S7>/Integrator2' */
  localDW->Integrator2_IWORK_n = 1;

  /* InitializeConditions for Integrator: '<S6>/Integrator1' incorporates:
   *  Integrator: '<S6>/Integrator2'
   */
  if (rtmIsFirstInitCond(mdl_MoCo_Brake_M)) {
    localX->Integrator1_CSTATE_a = 0.0;
    localX->Integrator2_CSTATE_o = 0.0;
  }

  localDW->Integrator1_IWORK_j = 1;

  /* End of InitializeConditions for Integrator: '<S6>/Integrator1' */

  /* InitializeConditions for Integrator: '<S6>/Integrator2' */
  localDW->Integrator2_IWORK_n0 = 1;

  /* InitializeConditions for TransferFcn: '<Root>/Filter1' */
  localX->Filter1_CSTATE = 0.0;

  /* InitializeConditions for Derivative: '<Root>/Derivative' */
  localDW->TimeStampA = (rtInf);
  localDW->TimeStampB = (rtInf);

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  localDW->PrevY = mdl_MoCo_Brake_P.RateLimiter_IC;

  /* InitializeConditions for Memory: '<Root>/Memory1' */
  localDW->Memory1_PreviousInput_o = mdl_MoCo_Brake_P.Memory1_InitialCondition_i;

  /* InitializeConditions for Memory: '<Root>/Memory2' */
  localDW->Memory2_PreviousInput = mdl_MoCo_Brake_P.Memory2_InitialCondition;

  /* SystemReset for Chart: '<Root>/Chart1' */
  localDW->temporalCounter_i1 = 0U;
  localDW->is_active_c3_mdl_MoCo_Brake = 0U;
  localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_NO_ACTIVE_CHILD;
}

/* Start for referenced model: 'mdl_MoCo_Brake' */
void mdl_MoCo_Brake_Start(RT_MODEL_mdl_MoCo_Brake_T * const mdl_MoCo_Brake_M,
  DW_mdl_MoCo_Brake_f_T *localDW)
{
  /* Start for TransportDelay: '<Root>/delay_brake_inc_init' */
  {
    real_T *pBuffer = &localDW->delay_brake_inc_init_RWORK.TUbufferArea[0];
    localDW->delay_brake_inc_init_IWORK.Tail[0] = 0;
    localDW->delay_brake_inc_init_IWORK.Head[0] = 0;
    localDW->delay_brake_inc_init_IWORK.Last[0] = 0;
    localDW->delay_brake_inc_init_IWORK.CircularBufSize[0] = 2048;
    pBuffer[0] = mdl_MoCo_Brake_P.delay_brake_inc_init_InitOutput;
    pBuffer[2048] = (*(mdl_MoCo_Brake_M->timingBridge->taskTime
                       [mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[0] = (void *) &pBuffer[0];
    localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[2] = (void *) &pBuffer[2048];
    pBuffer += 4096;
    localDW->delay_brake_inc_init_IWORK.Tail[1] = 0;
    localDW->delay_brake_inc_init_IWORK.Head[1] = 0;
    localDW->delay_brake_inc_init_IWORK.Last[1] = 0;
    localDW->delay_brake_inc_init_IWORK.CircularBufSize[1] = 2048;
    pBuffer[0] = mdl_MoCo_Brake_P.delay_brake_inc_init_InitOutput;
    pBuffer[2048] = (*(mdl_MoCo_Brake_M->timingBridge->taskTime
                       [mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[1] = (void *) &pBuffer[0];
    localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[3] = (void *) &pBuffer[2048];
  }

  /* Start for TransportDelay: '<Root>/delay_brake_inc' */
  {
    real_T *pBuffer = &localDW->delay_brake_inc_RWORK.TUbufferArea[0];
    localDW->delay_brake_inc_IWORK.Tail[0] = 0;
    localDW->delay_brake_inc_IWORK.Head[0] = 0;
    localDW->delay_brake_inc_IWORK.Last[0] = 0;
    localDW->delay_brake_inc_IWORK.CircularBufSize[0] = 2048;
    pBuffer[0] = mdl_MoCo_Brake_P.delay_brake_inc_InitOutput;
    pBuffer[2048] = (*(mdl_MoCo_Brake_M->timingBridge->taskTime
                       [mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    localDW->delay_brake_inc_PWORK.TUbufferPtrs[0] = (void *) &pBuffer[0];
    localDW->delay_brake_inc_PWORK.TUbufferPtrs[2] = (void *) &pBuffer[2048];
    pBuffer += 4096;
    localDW->delay_brake_inc_IWORK.Tail[1] = 0;
    localDW->delay_brake_inc_IWORK.Head[1] = 0;
    localDW->delay_brake_inc_IWORK.Last[1] = 0;
    localDW->delay_brake_inc_IWORK.CircularBufSize[1] = 2048;
    pBuffer[0] = mdl_MoCo_Brake_P.delay_brake_inc_InitOutput;
    pBuffer[2048] = (*(mdl_MoCo_Brake_M->timingBridge->taskTime
                       [mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    localDW->delay_brake_inc_PWORK.TUbufferPtrs[1] = (void *) &pBuffer[0];
    localDW->delay_brake_inc_PWORK.TUbufferPtrs[3] = (void *) &pBuffer[2048];
  }
}

/* Outputs for referenced model: 'mdl_MoCo_Brake' */
void mdl_MoCo_Brake(RT_MODEL_mdl_MoCo_Brake_T * const mdl_MoCo_Brake_M, const
                    real_T *rtu_SET_TRQ, const real_T *rtu_SSM_HOLD_REQ, const
                    real_T *rtu_SSM_GO, real_T *rty_ACCEL, real_T
                    *rty_Torque_Out, real_T *rty_SSM_Status, real_T
                    *rty_SSM_Torque, B_mdl_MoCo_Brake_c_T *localB,
                    DW_mdl_MoCo_Brake_f_T *localDW, X_mdl_MoCo_Brake_n_T *localX,
                    ZCE_mdl_MoCo_Brake_T *localZCE)
{
  ZCEventType zcEvent;
  real_T *lastU;
  real_T rateLimiterRate;
  real_T rtb_Min;
  real_T rtb_Derivative;
  real_T rtb_Add;
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Sum: '<Root>/Add' */
    rtb_Add = *rtu_SSM_GO + *rtu_SSM_HOLD_REQ;
  }

  /* TransportDelay: '<Root>/delay_brake_inc_init' */
  {
    real_T **uBuffer = (real_T**)
      &localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[0];
    real_T **tBuffer = (real_T**)
      &localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[2];
    real_T simTime = (*(mdl_MoCo_Brake_M->timingBridge->
                        taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    real_T tMinusDelay ;
    tMinusDelay = ((rtP_delay_brake_inc_init > 0.0) ? rtP_delay_brake_inc_init :
                   0.0);
    tMinusDelay = simTime - tMinusDelay;
    localB->delay_brake_inc_init[0] = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *tBuffer,
      *uBuffer,
      localDW->delay_brake_inc_init_IWORK.CircularBufSize[0],
      &localDW->delay_brake_inc_init_IWORK.Last[0],
      localDW->delay_brake_inc_init_IWORK.Tail[0],
      localDW->delay_brake_inc_init_IWORK.Head[0],
      mdl_MoCo_Brake_P.delay_brake_inc_init_InitOutput,
      1,
      0);
    tBuffer++;
    uBuffer++;
    tMinusDelay = ((rtP_delay_brake_inc_init > 0.0) ? rtP_delay_brake_inc_init :
                   0.0);
    tMinusDelay = simTime - tMinusDelay;
    localB->delay_brake_inc_init[1] = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *tBuffer,
      *uBuffer,
      localDW->delay_brake_inc_init_IWORK.CircularBufSize[1],
      &localDW->delay_brake_inc_init_IWORK.Last[1],
      localDW->delay_brake_inc_init_IWORK.Tail[1],
      localDW->delay_brake_inc_init_IWORK.Head[1],
      mdl_MoCo_Brake_P.delay_brake_inc_init_InitOutput,
      1,
      0);
  }

  /* TransportDelay: '<Root>/delay_brake_inc' */
  {
    real_T **uBuffer = (real_T**)&localDW->delay_brake_inc_PWORK.TUbufferPtrs[0];
    real_T **tBuffer = (real_T**)&localDW->delay_brake_inc_PWORK.TUbufferPtrs[2];
    real_T simTime = (*(mdl_MoCo_Brake_M->timingBridge->
                        taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    real_T tMinusDelay ;
    tMinusDelay = ((rtP_delay_brake_inc > 0.0) ? rtP_delay_brake_inc : 0.0);
    tMinusDelay = simTime - tMinusDelay;
    localB->delay_brake_inc[0] = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *tBuffer,
      *uBuffer,
      localDW->delay_brake_inc_IWORK.CircularBufSize[0],
      &localDW->delay_brake_inc_IWORK.Last[0],
      localDW->delay_brake_inc_IWORK.Tail[0],
      localDW->delay_brake_inc_IWORK.Head[0],
      mdl_MoCo_Brake_P.delay_brake_inc_InitOutput,
      1,
      0);
    tBuffer++;
    uBuffer++;
    tMinusDelay = ((rtP_delay_brake_inc > 0.0) ? rtP_delay_brake_inc : 0.0);
    tMinusDelay = simTime - tMinusDelay;
    localB->delay_brake_inc[1] = rt_TDelayInterpolate(
      tMinusDelay,
      0.0,
      *tBuffer,
      *uBuffer,
      localDW->delay_brake_inc_IWORK.CircularBufSize[1],
      &localDW->delay_brake_inc_IWORK.Last[1],
      localDW->delay_brake_inc_IWORK.Tail[1],
      localDW->delay_brake_inc_IWORK.Head[1],
      mdl_MoCo_Brake_P.delay_brake_inc_InitOutput,
      1,
      0);
  }

  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Memory: '<Root>/Memory3' */
    localB->brk_dec = localDW->Memory3_PreviousInput;

    /* Chart: '<Root>/Chart1' incorporates:
     *  Memory: '<Root>/Memory4'
     */
    if (localDW->temporalCounter_i1 < MAX_uint32_T) {
      localDW->temporalCounter_i1++;
    }

    if (localDW->is_active_c3_mdl_MoCo_Brake == 0U) {
      localDW->is_active_c3_mdl_MoCo_Brake = 1U;
      localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_dec;
      localB->outp = localB->brk_dec;
    } else {
      switch (localDW->is_c3_mdl_MoCo_Brake) {
       case mdl_MoCo_Brake_IN_Inc_Trans_init:
        if (localDW->temporalCounter_i1 >= (uint32_T)ceil
            (rtP_delay_brake_inc_init * 1000.0)) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_Inc_init;
        } else if (localDW->Memory4_PreviousInput == -1.0) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_dec;
          localB->outp = localB->brk_dec;
        } else {
          localB->outp = localDW->temp;
        }
        break;

       case mdl_MoCo_Brake_IN_Inc_init:
        if (localDW->Memory4_PreviousInput == -1.0) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_dec;
          localB->outp = localB->brk_dec;
        } else {
          localB->outp = localB->delay_brake_inc_init[0];
          localB->out_reset = localB->delay_brake_inc_init[1];
        }
        break;

       case mdl_MoCo_Brake_IN_Inc_Trans:
        if (localDW->temporalCounter_i1 >= (uint32_T)ceil(rtP_delay_brake_inc *
             1000.0)) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_Inc_Trans1;
        } else if (localDW->Memory4_PreviousInput == -1.0) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_dec;
          localB->outp = localB->brk_dec;
        } else {
          localB->outp = localDW->temp;
        }
        break;

       case mdl_MoCo_Brake_IN_Inc_Trans1:
        if (localDW->Memory4_PreviousInput == -1.0) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_dec;
          localB->outp = localB->brk_dec;
        } else {
          localB->outp = localB->delay_brake_inc[0];
          localB->out_reset = localB->delay_brake_inc[1];
        }
        break;

       default:
        /* case IN_dec: */
        if ((localDW->Memory4_PreviousInput == 1.0) && (localB->outp >
             rtP_brake_pres_exist_threshold)) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_Inc_Trans_init;
          localDW->temporalCounter_i1 = 0U;
          localDW->temp = localB->brk_dec;
        } else if ((localDW->Memory4_PreviousInput == 1.0) && (localB->outp <=
                    rtP_brake_pres_exist_threshold)) {
          localDW->is_c3_mdl_MoCo_Brake = mdl_MoCo_Brake_IN_Inc_Trans;
          localDW->temporalCounter_i1 = 0U;
          localDW->temp = localB->brk_dec;
        } else {
          localB->outp = localB->brk_dec;
        }
        break;
      }
    }

    /* End of Chart: '<Root>/Chart1' */

    /* Memory: '<Root>/Memory' */
    localB->Memory = localDW->Memory_PreviousInput;

    /* Gain: '<Root>/Gain3 [1//c]' incorporates:
     *  Gain: '<Root>/Gain4 [d]'
     *  Sum: '<Root>/Sum1'
     */
    localB->Gain31c = (1.0 / rtP_T_brk_accel_dec * localB->outp +
                       localB->out_reset) * (rtP_T_brk_accel_dec /
      rtP_K_brk_accel_dec);
  }

  /* Integrator: '<S2>/Integrator2' */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    zcEvent = rt_ZCFcn(FALLING_ZERO_CROSSING,&localZCE->Integrator2_Reset_ZCE,
                       (localB->Memory));

    /* evaluate zero-crossings */
    if ((zcEvent != NO_ZCEVENT) || (localDW->Integrator2_IWORK != 0)) {
      localX->Integrator2_CSTATE = localB->Gain31c;
    }
  }

  /* Sum: '<S2>/Sum2' incorporates:
   *  Gain: '<S2>/Gain [a]'
   *  Gain: '<S2>/Gain [b]'
   *  Integrator: '<S2>/Integrator2'
   */
  localB->Sum2 = rtP_K_brk_trq_dec / rtP_T_brk_trq_dec * *rtu_SET_TRQ - 1.0 /
    rtP_T_brk_trq_dec * localX->Integrator2_CSTATE;

  /* Integrator: '<S3>/Integrator2' */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    zcEvent = rt_ZCFcn(FALLING_ZERO_CROSSING,&localZCE->Integrator2_Reset_ZCE_m,
                       (localB->Memory));

    /* evaluate zero-crossings */
    if ((zcEvent != NO_ZCEVENT) || (localDW->Integrator2_IWORK_d != 0)) {
      localX->Integrator2_CSTATE_f = localB->outp;
    }
  }

  localB->Integrator2 = localX->Integrator2_CSTATE_f;

  /* End of Integrator: '<S3>/Integrator2' */

  /* Gain: '<S3>/Gain [c]' incorporates:
   *  Gain: '<Root>/Gain1 [c]'
   *  Integrator: '<S2>/Integrator2'
   */
  rtb_Derivative = rtP_K_brk_accel_dec / rtP_T_brk_accel_dec *
    localX->Integrator2_CSTATE;

  /* Sum: '<S3>/Sum2' incorporates:
   *  Gain: '<S3>/Gain [c]'
   *  Gain: '<S3>/Gain [d]'
   */
  localB->Sum2_l = rtb_Derivative - 1.0 / rtP_T_brk_accel_dec *
    localB->Integrator2;
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Product: '<S5>/Product2' incorporates:
     *  Constant: '<S5>/Constant'
     *  Constant: '<S5>/Constant1'
     *  Gain: '<S5>/Gain'
     *  Memory: '<S5>/Memory'
     *  Memory: '<S5>/Memory1'
     *  Product: '<S5>/Divide'
     *  Sum: '<S5>/Subtract'
     */
    rtb_Min = (mdl_MoCo_Brake_P.Gain_Gain * localDW->Memory1_PreviousInput[0] -
               mdl_MoCo_Brake_P.Constant_Value) /
      mdl_MoCo_Brake_P.Constant1_Value_b * localDW->Memory_PreviousInput_g;

    /* Saturate: '<S5>/Saturation1' */
    if (rtb_Min > mdl_MoCo_Brake_P.Saturation1_UpperSat) {
      localB->Saturation1 = mdl_MoCo_Brake_P.Saturation1_UpperSat;
    } else if (rtb_Min < mdl_MoCo_Brake_P.Saturation1_LowerSat) {
      localB->Saturation1 = mdl_MoCo_Brake_P.Saturation1_LowerSat;
    } else {
      localB->Saturation1 = rtb_Min;
    }

    /* End of Saturate: '<S5>/Saturation1' */

    /* Sum: '<S5>/Subtract1' incorporates:
     *  Constant: '<S5>/Constant2'
     */
    localB->Subtract1 = mdl_MoCo_Brake_P.Constant2_Value - localB->Saturation1;
  }

  /* Integrator: '<S7>/Integrator1' */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,&localZCE->Integrator1_Reset_ZCE,
                       (localB->Memory));

    /* evaluate zero-crossings */
    if ((zcEvent != NO_ZCEVENT) || (localDW->Integrator1_IWORK != 0)) {
      localX->Integrator1_CSTATE = localB->brk_dec;
    }
  }

  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Gain: '<Root>/Gain2 [d]' */
    localB->Gain2d = 1.0 / rtP_T_brk_accel_dec * localB->brk_dec;
  }

  /* Sum: '<Root>/Sum' */
  localB->Sum = rtb_Derivative - localB->Gain2d;

  /* Integrator: '<S7>/Integrator2' */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,&localZCE->Integrator2_Reset_ZCE_l,
                       (localB->Memory));

    /* evaluate zero-crossings */
    if ((zcEvent != NO_ZCEVENT) || (localDW->Integrator2_IWORK_n != 0)) {
      localX->Integrator2_CSTATE_d = localB->Sum;
    }
  }

  localB->Integrator2_i = localX->Integrator2_CSTATE_d;

  /* End of Integrator: '<S7>/Integrator2' */

  /* Integrator: '<S6>/Integrator1' incorporates:
   *  Integrator: '<S6>/Integrator2'
   */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,&localZCE->Integrator1_Reset_ZCE_o,
                       (localB->Memory));

    /* evaluate zero-crossings */
    if ((zcEvent != NO_ZCEVENT) || (localDW->Integrator1_IWORK_j != 0)) {
      localX->Integrator1_CSTATE_a = localB->brk_dec;
    }

    zcEvent = rt_ZCFcn(RISING_ZERO_CROSSING,&localZCE->Integrator2_Reset_ZCE_f,
                       (localB->Memory));

    /* evaluate zero-crossings */
    if ((zcEvent != NO_ZCEVENT) || (localDW->Integrator2_IWORK_n0 != 0)) {
      localX->Integrator2_CSTATE_o = localB->Sum;
    }
  }

  /* Integrator: '<S6>/Integrator2' */
  localB->Integrator2_j = localX->Integrator2_CSTATE_o;

  /* Sum: '<S5>/add' incorporates:
   *  Integrator: '<S6>/Integrator1'
   *  Integrator: '<S7>/Integrator1'
   *  Product: '<S5>/Product'
   *  Product: '<S5>/Product1'
   */
  localB->output[0] = localB->Subtract1 * localX->Integrator1_CSTATE +
    localB->Saturation1 * localX->Integrator1_CSTATE_a;
  localB->output[1] = localB->Subtract1 * localB->Integrator2_i +
    localB->Saturation1 * localB->Integrator2_j;
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Math: '<S6>/Square' incorporates:
     *  Constant: '<S6>/Constant1'
     *  Math: '<S6>/Square1'
     */
    rtb_Derivative = rtP_w0_brk * rtP_w0_brk;
    localB->Square = rtb_Derivative;

    /* Math: '<S6>/Square1' */
    localB->Square1 = rtb_Derivative;

    /* Math: '<S7>/Square' incorporates:
     *  Constant: '<S7>/Constant1'
     *  Math: '<S7>/Square1'
     */
    rtb_Derivative = rtP_w0_brkinit * rtP_w0_brkinit;
    localB->Square_k = rtb_Derivative;

    /* Math: '<S7>/Square1' */
    localB->Square1_f = rtb_Derivative;
  }

  /* Sum: '<S6>/Sum2' incorporates:
   *  Constant: '<S6>/Constant'
   *  Gain: '<S6>/Gain [f]'
   *  Integrator: '<S6>/Integrator1'
   *  Product: '<S6>/Multiply'
   *  Product: '<S6>/Multiply1'
   *  Sum: '<S6>/Sum1'
   */
  localB->Sum2_g = (rtP_K_brk * localB->Square * *rtu_SET_TRQ -
                    localX->Integrator1_CSTATE_a * localB->Square1) - 2.0 *
    rtP_D_brk * rtP_w0_brk * localB->Integrator2_j;

  /* Sum: '<S7>/Sum2' incorporates:
   *  Constant: '<S7>/Constant'
   *  Gain: '<S7>/Gain [f]'
   *  Integrator: '<S7>/Integrator1'
   *  Product: '<S7>/Multiply'
   *  Product: '<S7>/Multiply1'
   *  Sum: '<S7>/Sum1'
   */
  localB->Sum2_gk = (rtP_K_brk * localB->Square_k * *rtu_SET_TRQ -
                     localX->Integrator1_CSTATE * localB->Square1_f) - 2.0 *
    rtP_D_brk * rtP_w0_brkinit * localB->Integrator2_i;

  /* TransferFcn: '<Root>/Filter1' */
  localB->accel_deriv = 0.0;
  localB->accel_deriv += mdl_MoCo_Brake_P.Filter1_C * localX->Filter1_CSTATE;

  /* Derivative: '<Root>/Derivative' */
  rtb_Derivative = (*(mdl_MoCo_Brake_M->timingBridge->taskTime
                      [mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
  if ((localDW->TimeStampA >= rtb_Derivative) && (localDW->TimeStampB >=
       rtb_Derivative)) {
    rtb_Derivative = 0.0;
  } else {
    rtb_Min = localDW->TimeStampA;
    lastU = &localDW->LastUAtTimeA;
    if (localDW->TimeStampA < localDW->TimeStampB) {
      if (localDW->TimeStampB < rtb_Derivative) {
        rtb_Min = localDW->TimeStampB;
        lastU = &localDW->LastUAtTimeB;
      }
    } else {
      if (localDW->TimeStampA >= rtb_Derivative) {
        rtb_Min = localDW->TimeStampB;
        lastU = &localDW->LastUAtTimeB;
      }
    }

    rtb_Derivative = (localB->accel_deriv - *lastU) / (rtb_Derivative - rtb_Min);
  }

  /* End of Derivative: '<Root>/Derivative' */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Switch: '<Root>/Switch1' incorporates:
     *  Constant: '<Root>/Constant'
     *  Constant: '<Root>/Constant1'
     */
    if (*rtu_SSM_HOLD_REQ > mdl_MoCo_Brake_P.Switch1_Threshold) {
      rtb_Min = rtP_Brk_SSM_Decel_Lim;
    } else {
      rtb_Min = mdl_MoCo_Brake_P.Constant1_Value;
    }

    /* End of Switch: '<Root>/Switch1' */

    /* RateLimiter: '<Root>/Rate Limiter' */
    rateLimiterRate = rtb_Min - localDW->PrevY;
    if (rateLimiterRate > mdl_MoCo_Brake_P.RateLimiter_RisingLim) {
      *rty_SSM_Torque = localDW->PrevY + mdl_MoCo_Brake_P.RateLimiter_RisingLim;
    } else if (rateLimiterRate < mdl_MoCo_Brake_P.RateLimiter_FallingLim) {
      *rty_SSM_Torque = localDW->PrevY + mdl_MoCo_Brake_P.RateLimiter_FallingLim;
    } else {
      *rty_SSM_Torque = rtb_Min;
    }

    localDW->PrevY = *rty_SSM_Torque;

    /* End of RateLimiter: '<Root>/Rate Limiter' */

    /* MinMax: '<Root>/Min' */
    rtb_Min = fmin(*rty_SSM_Torque, localB->outp);

    /* Saturate: '<Root>/Saturation' */
    if (rtb_Min > mdl_MoCo_Brake_P.Saturation_UpperSat) {
      *rty_ACCEL = mdl_MoCo_Brake_P.Saturation_UpperSat;
    } else if (rtb_Min < mdl_MoCo_Brake_P.Saturation_LowerSat) {
      *rty_ACCEL = mdl_MoCo_Brake_P.Saturation_LowerSat;
    } else {
      *rty_ACCEL = rtb_Min;
    }

    /* End of Saturate: '<Root>/Saturation' */

    /* Gain: '<Root>/Gain' */
    *rty_Torque_Out = rtP_accel_to_trq * *rty_ACCEL;

    /* Gain: '<Root>/Gain4' */
    localB->Gain4 = mdl_MoCo_Brake_P.Gain4_Gain * localB->outp;

    /* Memory: '<Root>/Memory1' */
    localB->Memory1 = localDW->Memory1_PreviousInput_o;

    /* Memory: '<Root>/Memory2' */
    localB->Memory2 = localDW->Memory2_PreviousInput;

    /* Saturate: '<Root>/Saturation1' */
    if (rtb_Add > mdl_MoCo_Brake_P.Saturation1_UpperSat_l) {
      *rty_SSM_Status = mdl_MoCo_Brake_P.Saturation1_UpperSat_l;
    } else if (rtb_Add < mdl_MoCo_Brake_P.Saturation1_LowerSat_a) {
      *rty_SSM_Status = mdl_MoCo_Brake_P.Saturation1_LowerSat_a;
    } else {
      *rty_SSM_Status = rtb_Add;
    }

    /* End of Saturate: '<Root>/Saturation1' */
  }

  /* Signum: '<Root>/Sign' */
  if (rtb_Derivative < 0.0) {
    localB->brk_inc_dec_switch = -1.0;
  } else if (rtb_Derivative > 0.0) {
    localB->brk_inc_dec_switch = 1.0;
  } else if (rtb_Derivative == 0.0) {
    localB->brk_inc_dec_switch = 0.0;
  } else {
    localB->brk_inc_dec_switch = (rtNaN);
  }

  /* End of Signum: '<Root>/Sign' */
}

/* Update for referenced model: 'mdl_MoCo_Brake' */
void mdl_MoCo_Brake_Update(RT_MODEL_mdl_MoCo_Brake_T * const mdl_MoCo_Brake_M,
  B_mdl_MoCo_Brake_c_T *localB, DW_mdl_MoCo_Brake_f_T *localDW)
{
  real_T *lastU;

  /* Update for TransportDelay: '<Root>/delay_brake_inc_init' */
  {
    real_T **uBuffer = (real_T**)
      &localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[0];
    real_T **tBuffer = (real_T**)
      &localDW->delay_brake_inc_init_PWORK.TUbufferPtrs[2];
    real_T simTime = (*(mdl_MoCo_Brake_M->timingBridge->
                        taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    localDW->delay_brake_inc_init_IWORK.Head[0] =
      ((localDW->delay_brake_inc_init_IWORK.Head[0] <
        (localDW->delay_brake_inc_init_IWORK.CircularBufSize[0]-1)) ?
       (localDW->delay_brake_inc_init_IWORK.Head[0]+1) : 0);
    if (localDW->delay_brake_inc_init_IWORK.Head[0] ==
        localDW->delay_brake_inc_init_IWORK.Tail[0]) {
      localDW->delay_brake_inc_init_IWORK.Tail[0] =
        ((localDW->delay_brake_inc_init_IWORK.Tail[0] <
          (localDW->delay_brake_inc_init_IWORK.CircularBufSize[0]-1)) ?
         (localDW->delay_brake_inc_init_IWORK.Tail[0]+1) : 0);
    }

    (*tBuffer++)[localDW->delay_brake_inc_init_IWORK.Head[0]] = simTime;
    (*uBuffer++)[localDW->delay_brake_inc_init_IWORK.Head[0]] = localB->Memory1;
    localDW->delay_brake_inc_init_IWORK.Head[1] =
      ((localDW->delay_brake_inc_init_IWORK.Head[1] <
        (localDW->delay_brake_inc_init_IWORK.CircularBufSize[1]-1)) ?
       (localDW->delay_brake_inc_init_IWORK.Head[1]+1) : 0);
    if (localDW->delay_brake_inc_init_IWORK.Head[1] ==
        localDW->delay_brake_inc_init_IWORK.Tail[1]) {
      localDW->delay_brake_inc_init_IWORK.Tail[1] =
        ((localDW->delay_brake_inc_init_IWORK.Tail[1] <
          (localDW->delay_brake_inc_init_IWORK.CircularBufSize[1]-1)) ?
         (localDW->delay_brake_inc_init_IWORK.Tail[1]+1) : 0);
    }

    (*tBuffer)[localDW->delay_brake_inc_init_IWORK.Head[1]] = simTime;
    (*uBuffer)[localDW->delay_brake_inc_init_IWORK.Head[1]] = localB->Memory2;
  }

  /* Update for TransportDelay: '<Root>/delay_brake_inc' */
  {
    real_T **uBuffer = (real_T**)&localDW->delay_brake_inc_PWORK.TUbufferPtrs[0];
    real_T **tBuffer = (real_T**)&localDW->delay_brake_inc_PWORK.TUbufferPtrs[2];
    real_T simTime = (*(mdl_MoCo_Brake_M->timingBridge->
                        taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    localDW->delay_brake_inc_IWORK.Head[0] =
      ((localDW->delay_brake_inc_IWORK.Head[0] <
        (localDW->delay_brake_inc_IWORK.CircularBufSize[0]-1)) ?
       (localDW->delay_brake_inc_IWORK.Head[0]+1) : 0);
    if (localDW->delay_brake_inc_IWORK.Head[0] ==
        localDW->delay_brake_inc_IWORK.Tail[0]) {
      localDW->delay_brake_inc_IWORK.Tail[0] =
        ((localDW->delay_brake_inc_IWORK.Tail[0] <
          (localDW->delay_brake_inc_IWORK.CircularBufSize[0]-1)) ?
         (localDW->delay_brake_inc_IWORK.Tail[0]+1) : 0);
    }

    (*tBuffer++)[localDW->delay_brake_inc_IWORK.Head[0]] = simTime;
    (*uBuffer++)[localDW->delay_brake_inc_IWORK.Head[0]] = localB->Memory1;
    localDW->delay_brake_inc_IWORK.Head[1] =
      ((localDW->delay_brake_inc_IWORK.Head[1] <
        (localDW->delay_brake_inc_IWORK.CircularBufSize[1]-1)) ?
       (localDW->delay_brake_inc_IWORK.Head[1]+1) : 0);
    if (localDW->delay_brake_inc_IWORK.Head[1] ==
        localDW->delay_brake_inc_IWORK.Tail[1]) {
      localDW->delay_brake_inc_IWORK.Tail[1] =
        ((localDW->delay_brake_inc_IWORK.Tail[1] <
          (localDW->delay_brake_inc_IWORK.CircularBufSize[1]-1)) ?
         (localDW->delay_brake_inc_IWORK.Tail[1]+1) : 0);
    }

    (*tBuffer)[localDW->delay_brake_inc_IWORK.Head[1]] = simTime;
    (*uBuffer)[localDW->delay_brake_inc_IWORK.Head[1]] = localB->Memory2;
  }

  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Update for Memory: '<Root>/Memory3' */
    localDW->Memory3_PreviousInput = localB->Integrator2;

    /* Update for Memory: '<Root>/Memory4' */
    localDW->Memory4_PreviousInput = localB->Memory;

    /* Update for Memory: '<Root>/Memory' */
    localDW->Memory_PreviousInput = localB->brk_inc_dec_switch;

    /* Update for Memory: '<S5>/Memory1' */
    localDW->Memory1_PreviousInput[0] = localB->output[0];
    localDW->Memory1_PreviousInput[1] = localB->output[1];

    /* Update for Memory: '<S5>/Memory' */
    localDW->Memory_PreviousInput_g = localB->Memory;
  }

  /* Update for Integrator: '<S2>/Integrator2' */
  localDW->Integrator2_IWORK = 0;

  /* Update for Integrator: '<S3>/Integrator2' */
  localDW->Integrator2_IWORK_d = 0;

  /* Update for Integrator: '<S7>/Integrator1' */
  localDW->Integrator1_IWORK = 0;

  /* Update for Integrator: '<S7>/Integrator2' */
  localDW->Integrator2_IWORK_n = 0;

  /* Update for Integrator: '<S6>/Integrator1' */
  localDW->Integrator1_IWORK_j = 0;

  /* Update for Integrator: '<S6>/Integrator2' */
  localDW->Integrator2_IWORK_n0 = 0;

  /* Update for Derivative: '<Root>/Derivative' */
  if (localDW->TimeStampA == (rtInf)) {
    localDW->TimeStampA = (*(mdl_MoCo_Brake_M->timingBridge->
      taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA;
  } else if (localDW->TimeStampB == (rtInf)) {
    localDW->TimeStampB = (*(mdl_MoCo_Brake_M->timingBridge->
      taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB;
  } else if (localDW->TimeStampA < localDW->TimeStampB) {
    localDW->TimeStampA = (*(mdl_MoCo_Brake_M->timingBridge->
      taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeA;
  } else {
    localDW->TimeStampB = (*(mdl_MoCo_Brake_M->timingBridge->
      taskTime[mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0]]));
    lastU = &localDW->LastUAtTimeB;
  }

  *lastU = localB->accel_deriv;

  /* End of Update for Derivative: '<Root>/Derivative' */
  if (rtmIsMajorTimeStep(mdl_MoCo_Brake_M)) {
    /* Update for Memory: '<Root>/Memory1' */
    localDW->Memory1_PreviousInput_o = localB->output[0];

    /* Update for Memory: '<Root>/Memory2' */
    localDW->Memory2_PreviousInput = localB->output[1];
  }
}

/* Derivatives for referenced model: 'mdl_MoCo_Brake' */
void mdl_MoCo_Brake_Deriv(B_mdl_MoCo_Brake_c_T *localB, X_mdl_MoCo_Brake_n_T
  *localX, XDot_mdl_MoCo_Brake_n_T *localXdot)
{
  /* Derivatives for Integrator: '<S2>/Integrator2' */
  localXdot->Integrator2_CSTATE = localB->Sum2;

  /* Derivatives for Integrator: '<S3>/Integrator2' */
  localXdot->Integrator2_CSTATE_f = localB->Sum2_l;

  /* Derivatives for Integrator: '<S7>/Integrator1' */
  localXdot->Integrator1_CSTATE = localB->Integrator2_i;

  /* Derivatives for Integrator: '<S7>/Integrator2' */
  localXdot->Integrator2_CSTATE_d = localB->Sum2_gk;

  /* Derivatives for Integrator: '<S6>/Integrator1' */
  localXdot->Integrator1_CSTATE_a = localB->Integrator2_j;

  /* Derivatives for Integrator: '<S6>/Integrator2' */
  localXdot->Integrator2_CSTATE_o = localB->Sum2_g;

  /* Derivatives for TransferFcn: '<Root>/Filter1' */
  localXdot->Filter1_CSTATE = 0.0;
  localXdot->Filter1_CSTATE += mdl_MoCo_Brake_P.Filter1_A *
    localX->Filter1_CSTATE;
  localXdot->Filter1_CSTATE += localB->Gain4;
}

/* Model initialize function */
void mdl_MoCo_Brake_initialize(const char_T **rt_errorStatus, boolean_T
  *rt_stopRequested, RTWSolverInfo *rt_solverInfo, const rtTimingBridge
  *timingBridge, int_T mdlref_TID0, int_T mdlref_TID1, RT_MODEL_mdl_MoCo_Brake_T
  *const mdl_MoCo_Brake_M, B_mdl_MoCo_Brake_c_T *localB, DW_mdl_MoCo_Brake_f_T
  *localDW, X_mdl_MoCo_Brake_n_T *localX, ZCE_mdl_MoCo_Brake_T *localZCE,
  rtwCAPI_ModelMappingInfo *rt_ParentMMI, const char_T *rt_ChildPath, int_T
  rt_ChildMMIIdx, int_T rt_CSTATEIdx)
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  /* non-finite (run-time) assignments */
  mdl_MoCo_Brake_P.Saturation_LowerSat = rtMinusInf;

  /* initialize real-time model */
  (void) memset((void *)mdl_MoCo_Brake_M, 0,
                sizeof(RT_MODEL_mdl_MoCo_Brake_T));

  /* setup the global timing engine */
  mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[0] = mdlref_TID0;
  mdl_MoCo_Brake_M->Timing.mdlref_GlobalTID[1] = mdlref_TID1;
  mdl_MoCo_Brake_M->timingBridge = (timingBridge);

  /* initialize error status */
  rtmSetErrorStatusPointer(mdl_MoCo_Brake_M, rt_errorStatus);

  /* initialize stop requested flag */
  rtmSetStopRequestedPtr(mdl_MoCo_Brake_M, rt_stopRequested);

  /* initialize RTWSolverInfo */
  mdl_MoCo_Brake_M->solverInfo = (rt_solverInfo);

  /* Set the Timing fields to the appropriate data in the RTWSolverInfo */
  rtmSetSimTimeStepPointer(mdl_MoCo_Brake_M, rtsiGetSimTimeStepPtr
    (mdl_MoCo_Brake_M->solverInfo));
  mdl_MoCo_Brake_M->Timing.stepSize0 = (rtsiGetStepSize
    (mdl_MoCo_Brake_M->solverInfo));

  /* block I/O */
  (void) memset(((void *) localB), 0,
                sizeof(B_mdl_MoCo_Brake_c_T));

  /* states (dwork) */
  (void) memset((void *)localDW, 0,
                sizeof(DW_mdl_MoCo_Brake_f_T));

  /* Initialize DataMapInfo substructure containing ModelMap for C API */
  {
    mdl_MoCo_Brake_InitializeDataMapInfo(mdl_MoCo_Brake_M, localB, localX);
  }

  /* Initialize Parent model MMI */
  if ((rt_ParentMMI != (NULL)) && (rt_ChildPath != (NULL))) {
    rtwCAPI_SetChildMMI(*rt_ParentMMI, rt_ChildMMIIdx,
                        &(mdl_MoCo_Brake_M->DataMapInfo.mmi));
    rtwCAPI_SetPath(mdl_MoCo_Brake_M->DataMapInfo.mmi, rt_ChildPath);
    rtwCAPI_MMISetContStateStartIndex(mdl_MoCo_Brake_M->DataMapInfo.mmi,
      rt_CSTATEIdx);
  }

  localZCE->Integrator2_Reset_ZCE = UNINITIALIZED_ZCSIG;
  localZCE->Integrator2_Reset_ZCE_m = UNINITIALIZED_ZCSIG;
  localZCE->Integrator1_Reset_ZCE = UNINITIALIZED_ZCSIG;
  localZCE->Integrator2_Reset_ZCE_l = UNINITIALIZED_ZCSIG;
  localZCE->Integrator1_Reset_ZCE_o = UNINITIALIZED_ZCSIG;
  localZCE->Integrator2_Reset_ZCE_f = UNINITIALIZED_ZCSIG;
}

/* CarMaker dictionary definitions. */
tQuantEntry mdl_MoCo_Brake_DictDefines[] = {
  { NULL, NULL, NULL, 0, 0, 0, "", 0.0, 0.0 }
};

/* CarMaker bodyframe definitions. */
#ifndef RTMAKER

tMdlBdyFrame mdl_MoCo_Brake_BdyFrameDefines[] = {
  { NULL }
};

#endif
