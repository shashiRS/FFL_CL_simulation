/*
 * rt_TDelayInterpolate.h
 *
 * Code generation for model "mdl_MoCo_Brake".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Jan 12 13:51:23 2022
 * Created for block: <Root>/delay_brake_inc_init
 */

#ifndef SHARE_rt_TDelayInterpolate
#define SHARE_rt_TDelayInterpolate
#include "rtwtypes.h"

real_T rt_TDelayInterpolate(
  real_T tMinusDelay,                 /* tMinusDelay = currentSimTime - delay */
  real_T tStart,
  real_T *tBuf,
  real_T *uBuf,
  int_T bufSz,
  int_T *lastIdx,
  int_T oldestIdx,
  int_T newIdx,
  real_T initOutput,
  boolean_T discrete,
  boolean_T minorStepAndTAtLastMajorOutput)
  ;

#endif
