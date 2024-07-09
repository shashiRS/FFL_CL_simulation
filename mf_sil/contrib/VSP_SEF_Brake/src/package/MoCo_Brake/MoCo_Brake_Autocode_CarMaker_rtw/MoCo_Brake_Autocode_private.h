/*
 * MoCo_Brake_Autocode_private.h
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

#ifndef RTW_HEADER_MoCo_Brake_Autocode_private_h_
#define RTW_HEADER_MoCo_Brake_Autocode_private_h_
#include "rtwtypes.h"
#include "model_reference_types.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"

/* Private macros used by the generated code to access rtModel */
#ifndef rtmSetFirstInitCond
# define rtmSetFirstInitCond(rtm, val) ((rtm)->Timing.firstInitCondFlag = (val))
#endif

#ifndef rtmIsFirstInitCond
# define rtmIsFirstInitCond(rtm)       ((rtm)->Timing.firstInitCondFlag)
#endif

#ifndef rtmIsMajorTimeStep
# define rtmIsMajorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MAJOR_TIME_STEP)
#endif

#ifndef rtmIsMinorTimeStep
# define rtmIsMinorTimeStep(rtm)       (((rtm)->Timing.simTimeStep) == MINOR_TIME_STEP)
#endif

#ifndef rtmSetTPtr
# define rtmSetTPtr(rtm, val)          ((rtm)->Timing.t = (val))
#endif

#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(0) { }
#endif

#define rt_FREE(ptr)                   if((ptr) != (NULL)) { free((void *)(ptr)); (ptr) = (NULL); }

/* private model entry point functions */
extern void MoCo_Brake_Autocode_derivatives(RT_MODEL_MoCo_Brake_Autocode_T *
  const MoCo_Brake_Autocode_M);

#endif                           /* RTW_HEADER_MoCo_Brake_Autocode_private_h_ */
