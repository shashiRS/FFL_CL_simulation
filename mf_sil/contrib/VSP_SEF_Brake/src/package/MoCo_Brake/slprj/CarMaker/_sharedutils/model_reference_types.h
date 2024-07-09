/*
 * model_reference_types.h
 *
 * Code generation for model "mdl_MoCo_Brake".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Jan 12 13:51:23 2022
 */

#ifndef MODEL_REFERENCE_TYPES_H
#define MODEL_REFERENCE_TYPES_H
#include "rtwtypes.h"
#ifndef MODEL_REFERENCE_TYPES
#define MODEL_REFERENCE_TYPES

/*===========================================================================*
 * Model reference type definitions                                          *
 *===========================================================================*/
/*
 * This structure is used by model reference to
 * communicate timing information through the hierarchy.
 */
typedef struct _rtTimingBridge_tag rtTimingBridge;
struct _rtTimingBridge_tag {
  uint32_T nTasks;
  uint32_T** clockTick;
  uint32_T** clockTickH;
  uint32_T* taskCounter;
  real_T** taskTime;
  boolean_T** rateTransition;
  boolean_T *firstInitCond;
};

/*
 * This structure is used by model reference to
 * communicate variable discrete rate timing information through the hierarchy.
 */
typedef struct _rtCtrlRateMdlRefTiming_tag rtCtrlRateMdlRefTiming;
struct _rtCtrlRateMdlRefTiming_tag {
  uint32_T firstCtrlRateTID;
  uint32_T* numTicksToNextHitForCtrlRate;
};

#endif                                 /* MODEL_REFERENCE_TYPES */
#endif                                 /* MODEL_REFERENCE_TYPES_H */
