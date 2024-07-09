/*
 * mdl_MoCo_Brake_capi.c
 *
 * Code generation for model "mdl_MoCo_Brake".
 *
 * Model version              : 1.3
 * Simulink Coder version : 9.2 (R2019b) 18-Jul-2019
 * C source code generated on : Wed Jan 12 13:51:23 2022
 *
 * Target selection: CarMaker.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "mdl_MoCo_Brake_capi_host.h"
#define sizeof(s)                      ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#else                                  /* HOST_CAPI_BUILD */
#include "builtin_typeid_types.h"
#include "mdl_MoCo_Brake.h"
#include "mdl_MoCo_Brake_capi.h"
#include "mdl_MoCo_Brake_private.h"
#ifdef LIGHT_WEIGHT_CAPI
#define TARGET_CONST
#define TARGET_STRING(s)               (NULL)
#else
#define TARGET_CONST                   const
#define TARGET_STRING(s)               (s)
#endif
#endif                                 /* HOST_CAPI_BUILD */

/* Block output signal information */
static rtwCAPI_Signals rtBlockSignals[] = {
  /* addrMapIndex, sysNum, blockPath,
   * signalName, portNumber, dataTypeIndex, dimIndex, fxpIndex, sTimeIndex
   */
  { 0, 3, TARGET_STRING("mdl_MoCo_Brake/Chart1"),
    TARGET_STRING("brk"), 0, 0, 0, 0, 0 },

  { 1, 3, TARGET_STRING("mdl_MoCo_Brake/Chart1"),
    TARGET_STRING(""), 1, 0, 0, 0, 0 },

  { 2, 4, TARGET_STRING("mdl_MoCo_Brake/Gain2 [d]"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 3, 4, TARGET_STRING("mdl_MoCo_Brake/Gain3 [1//c]"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 4, 4, TARGET_STRING("mdl_MoCo_Brake/Gain4"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 5, 4, TARGET_STRING("mdl_MoCo_Brake/Memory"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 6, 4, TARGET_STRING("mdl_MoCo_Brake/Memory1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 7, 4, TARGET_STRING("mdl_MoCo_Brake/Memory2"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 8, 4, TARGET_STRING("mdl_MoCo_Brake/Memory3"),
    TARGET_STRING("brk_dec"), 0, 0, 0, 0, 0 },

  { 9, 4, TARGET_STRING("mdl_MoCo_Brake/Sign"),
    TARGET_STRING("brk_inc_dec_switch"), 0, 0, 0, 0, 1 },

  { 10, 4, TARGET_STRING("mdl_MoCo_Brake/Sum"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 11, 4, TARGET_STRING("mdl_MoCo_Brake/Filter1"),
    TARGET_STRING("accel_deriv"), 0, 0, 0, 0, 1 },

  { 12, 4, TARGET_STRING("mdl_MoCo_Brake/delay_brake_inc"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 13, 4, TARGET_STRING("mdl_MoCo_Brake/delay_brake_inc_init"),
    TARGET_STRING(""), 0, 0, 1, 0, 1 },

  { 14, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT1_first/Sum2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 15, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT1_second/Integrator2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 16, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT1_second/Sum2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 17, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Saturation1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 18, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Subtract1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 19, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/add"),
    TARGET_STRING("output"), 0, 0, 1, 0, 1 },

  { 20, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2/Integrator2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 21, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2/Square"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 22, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2/Square1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 23, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2/Sum2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 24, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2_init/Integrator2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  { 25, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2_init/Square"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 26, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2_init/Square1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 27, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2_init/Sum2"),
    TARGET_STRING(""), 0, 0, 0, 0, 1 },

  {
    0, 0, (NULL), (NULL), 0, 0, 0, 0, 0
  }
};

static rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  { 28, TARGET_STRING("mdl_MoCo_Brake/Constant1"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 29, TARGET_STRING("mdl_MoCo_Brake/Gain4"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 30, TARGET_STRING("mdl_MoCo_Brake/Memory"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 31, TARGET_STRING("mdl_MoCo_Brake/Memory1"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 32, TARGET_STRING("mdl_MoCo_Brake/Memory2"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 33, TARGET_STRING("mdl_MoCo_Brake/Memory3"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 34, TARGET_STRING("mdl_MoCo_Brake/Memory4"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 35, TARGET_STRING("mdl_MoCo_Brake/Rate Limiter"),
    TARGET_STRING("RisingSlewLimit"), 0, 0, 0 },

  { 36, TARGET_STRING("mdl_MoCo_Brake/Rate Limiter"),
    TARGET_STRING("FallingSlewLimit"), 0, 0, 0 },

  { 37, TARGET_STRING("mdl_MoCo_Brake/Rate Limiter"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 38, TARGET_STRING("mdl_MoCo_Brake/Saturation"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 39, TARGET_STRING("mdl_MoCo_Brake/Saturation"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 40, TARGET_STRING("mdl_MoCo_Brake/Saturation1"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 41, TARGET_STRING("mdl_MoCo_Brake/Saturation1"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  { 42, TARGET_STRING("mdl_MoCo_Brake/Switch1"),
    TARGET_STRING("Threshold"), 0, 0, 0 },

  { 43, TARGET_STRING("mdl_MoCo_Brake/Filter1"),
    TARGET_STRING("A"), 0, 0, 0 },

  { 44, TARGET_STRING("mdl_MoCo_Brake/Filter1"),
    TARGET_STRING("C"), 0, 0, 0 },

  { 45, TARGET_STRING("mdl_MoCo_Brake/delay_brake_inc"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 46, TARGET_STRING("mdl_MoCo_Brake/delay_brake_inc_init"),
    TARGET_STRING("InitialOutput"), 0, 0, 0 },

  { 47, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Constant"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 48, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Constant1"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 49, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Constant2"),
    TARGET_STRING("Value"), 0, 0, 0 },

  { 50, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Gain"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 51, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Memory"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 52, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Memory1"),
    TARGET_STRING("InitialCondition"), 0, 0, 0 },

  { 53, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Saturation1"),
    TARGET_STRING("UpperLimit"), 0, 0, 0 },

  { 54, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/Blending/Saturation1"),
    TARGET_STRING("LowerLimit"), 0, 0, 0 },

  {
    0, (NULL), (NULL), 0, 0, 0
  }
};

/* Block states information */
static rtwCAPI_States rtBlockStates[] = {
  /* addrMapIndex, contStateStartIndex, blockPath,
   * stateName, pathAlias, dWorkIndex, dataTypeIndex, dimIndex,
   * fixPtIdx, sTimeIndex, isContinuous, hierInfoIdx, flatElemIdx
   */
  { 55, 6, TARGET_STRING("mdl_MoCo_Brake/Filter1"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  { 56, 0, TARGET_STRING("mdl_MoCo_Brake/DEC_PT1_first/Integrator2"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  { 57, 1, TARGET_STRING("mdl_MoCo_Brake/DEC_PT1_second/Integrator2"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  { 58, 4, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2/Integrator1"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  { 59, 5, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2/Integrator2"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  { 60, 2, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2_init/Integrator1"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  { 61, 3, TARGET_STRING("mdl_MoCo_Brake/DEC_PT2/INC_PT2_init/Integrator2"),
    TARGET_STRING(""),
    TARGET_STRING(""),
    0, 0, 0, 0, 1, 1, -1, 0 },

  {
    0, -1, (NULL), (NULL), (NULL), 0, 0, 0, 0, 0, 0, -1, 0
  }
};

/* Tunable variable parameters */
static rtwCAPI_ModelParameters rtModelParameters[] = {
  /* addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex */
  { 62, TARGET_STRING("Brk_SSM_Decel_Lim"), 0, 0, 0 },

  { 63, TARGET_STRING("D_brk"), 0, 0, 0 },

  { 64, TARGET_STRING("K_brk"), 0, 0, 0 },

  { 65, TARGET_STRING("K_brk_accel_dec"), 0, 0, 0 },

  { 66, TARGET_STRING("K_brk_trq_dec"), 0, 0, 0 },

  { 67, TARGET_STRING("T_brk_accel_dec"), 0, 0, 0 },

  { 68, TARGET_STRING("T_brk_trq_dec"), 0, 0, 0 },

  { 69, TARGET_STRING("accel_to_trq"), 0, 0, 0 },

  { 70, TARGET_STRING("brake_pres_exist_threshold"), 0, 0, 0 },

  { 71, TARGET_STRING("delay_brake_inc"), 0, 0, 0 },

  { 72, TARGET_STRING("delay_brake_inc_init"), 0, 0, 0 },

  { 73, TARGET_STRING("w0_brk"), 0, 0, 0 },

  { 74, TARGET_STRING("w0_brkinit"), 0, 0, 0 },

  { 0, (NULL), 0, 0, 0 }
};

#ifndef HOST_CAPI_BUILD

/* Initialize Data Address */
static void mdl_MoCo_Brake_InitializeDataAddr(void* dataAddr[],
  B_mdl_MoCo_Brake_c_T *localB, X_mdl_MoCo_Brake_n_T *localX)
{
  dataAddr[0] = (void*) (&localB->outp);
  dataAddr[1] = (void*) (&localB->out_reset);
  dataAddr[2] = (void*) (&localB->Gain2d);
  dataAddr[3] = (void*) (&localB->Gain31c);
  dataAddr[4] = (void*) (&localB->Gain4);
  dataAddr[5] = (void*) (&localB->Memory);
  dataAddr[6] = (void*) (&localB->Memory1);
  dataAddr[7] = (void*) (&localB->Memory2);
  dataAddr[8] = (void*) (&localB->brk_dec);
  dataAddr[9] = (void*) (&localB->brk_inc_dec_switch);
  dataAddr[10] = (void*) (&localB->Sum);
  dataAddr[11] = (void*) (&localB->accel_deriv);
  dataAddr[12] = (void*) (&localB->delay_brake_inc[0]);
  dataAddr[13] = (void*) (&localB->delay_brake_inc_init[0]);
  dataAddr[14] = (void*) (&localB->Sum2);
  dataAddr[15] = (void*) (&localB->Integrator2);
  dataAddr[16] = (void*) (&localB->Sum2_l);
  dataAddr[17] = (void*) (&localB->Saturation1);
  dataAddr[18] = (void*) (&localB->Subtract1);
  dataAddr[19] = (void*) (&localB->output[0]);
  dataAddr[20] = (void*) (&localB->Integrator2_j);
  dataAddr[21] = (void*) (&localB->Square);
  dataAddr[22] = (void*) (&localB->Square1);
  dataAddr[23] = (void*) (&localB->Sum2_g);
  dataAddr[24] = (void*) (&localB->Integrator2_i);
  dataAddr[25] = (void*) (&localB->Square_k);
  dataAddr[26] = (void*) (&localB->Square1_f);
  dataAddr[27] = (void*) (&localB->Sum2_gk);
  dataAddr[28] = (void*) (&mdl_MoCo_Brake_P.Constant1_Value);
  dataAddr[29] = (void*) (&mdl_MoCo_Brake_P.Gain4_Gain);
  dataAddr[30] = (void*) (&mdl_MoCo_Brake_P.Memory_InitialCondition);
  dataAddr[31] = (void*) (&mdl_MoCo_Brake_P.Memory1_InitialCondition_i);
  dataAddr[32] = (void*) (&mdl_MoCo_Brake_P.Memory2_InitialCondition);
  dataAddr[33] = (void*) (&mdl_MoCo_Brake_P.Memory3_InitialCondition);
  dataAddr[34] = (void*) (&mdl_MoCo_Brake_P.Memory4_InitialCondition);
  dataAddr[35] = (void*) (&mdl_MoCo_Brake_P.RateLimiter_RisingLim);
  dataAddr[36] = (void*) (&mdl_MoCo_Brake_P.RateLimiter_FallingLim);
  dataAddr[37] = (void*) (&mdl_MoCo_Brake_P.RateLimiter_IC);
  dataAddr[38] = (void*) (&mdl_MoCo_Brake_P.Saturation_UpperSat);
  dataAddr[39] = (void*) (&mdl_MoCo_Brake_P.Saturation_LowerSat);
  dataAddr[40] = (void*) (&mdl_MoCo_Brake_P.Saturation1_UpperSat_l);
  dataAddr[41] = (void*) (&mdl_MoCo_Brake_P.Saturation1_LowerSat_a);
  dataAddr[42] = (void*) (&mdl_MoCo_Brake_P.Switch1_Threshold);
  dataAddr[43] = (void*) (&mdl_MoCo_Brake_P.Filter1_A);
  dataAddr[44] = (void*) (&mdl_MoCo_Brake_P.Filter1_C);
  dataAddr[45] = (void*) (&mdl_MoCo_Brake_P.delay_brake_inc_InitOutput);
  dataAddr[46] = (void*) (&mdl_MoCo_Brake_P.delay_brake_inc_init_InitOutput);
  dataAddr[47] = (void*) (&mdl_MoCo_Brake_P.Constant_Value);
  dataAddr[48] = (void*) (&mdl_MoCo_Brake_P.Constant1_Value_b);
  dataAddr[49] = (void*) (&mdl_MoCo_Brake_P.Constant2_Value);
  dataAddr[50] = (void*) (&mdl_MoCo_Brake_P.Gain_Gain);
  dataAddr[51] = (void*) (&mdl_MoCo_Brake_P.Memory_InitialCondition_g);
  dataAddr[52] = (void*) (&mdl_MoCo_Brake_P.Memory1_InitialCondition);
  dataAddr[53] = (void*) (&mdl_MoCo_Brake_P.Saturation1_UpperSat);
  dataAddr[54] = (void*) (&mdl_MoCo_Brake_P.Saturation1_LowerSat);
  dataAddr[55] = (void*) (&localX->Filter1_CSTATE);
  dataAddr[56] = (void*) (&localX->Integrator2_CSTATE);
  dataAddr[57] = (void*) (&localX->Integrator2_CSTATE_f);
  dataAddr[58] = (void*) (&localX->Integrator1_CSTATE_a);
  dataAddr[59] = (void*) (&localX->Integrator2_CSTATE_o);
  dataAddr[60] = (void*) (&localX->Integrator1_CSTATE);
  dataAddr[61] = (void*) (&localX->Integrator2_CSTATE_d);
  dataAddr[62] = (void*) (&rtP_Brk_SSM_Decel_Lim);
  dataAddr[63] = (void*) (&rtP_D_brk);
  dataAddr[64] = (void*) (&rtP_K_brk);
  dataAddr[65] = (void*) (&rtP_K_brk_accel_dec);
  dataAddr[66] = (void*) (&rtP_K_brk_trq_dec);
  dataAddr[67] = (void*) (&rtP_T_brk_accel_dec);
  dataAddr[68] = (void*) (&rtP_T_brk_trq_dec);
  dataAddr[69] = (void*) (&rtP_accel_to_trq);
  dataAddr[70] = (void*) (&rtP_brake_pres_exist_threshold);
  dataAddr[71] = (void*) (&rtP_delay_brake_inc);
  dataAddr[72] = (void*) (&rtP_delay_brake_inc_init);
  dataAddr[73] = (void*) (&rtP_w0_brk);
  dataAddr[74] = (void*) (&rtP_w0_brkinit);
}

#endif

/* Initialize Data Run-Time Dimension Buffer Address */
#ifndef HOST_CAPI_BUILD

static void mdl_MoCo_Brake_InitializeVarDimsAddr(int32_T* vardimsAddr[])
{
  vardimsAddr[0] = (NULL);
}

#endif

#ifndef HOST_CAPI_BUILD

/* Initialize logging function pointers */
static void mdl_MoCo_Brake_InitializeLoggingFunctions(RTWLoggingFcnPtr
  loggingPtrs[])
{
  loggingPtrs[0] = (NULL);
  loggingPtrs[1] = (NULL);
  loggingPtrs[2] = (NULL);
  loggingPtrs[3] = (NULL);
  loggingPtrs[4] = (NULL);
  loggingPtrs[5] = (NULL);
  loggingPtrs[6] = (NULL);
  loggingPtrs[7] = (NULL);
  loggingPtrs[8] = (NULL);
  loggingPtrs[9] = (NULL);
  loggingPtrs[10] = (NULL);
  loggingPtrs[11] = (NULL);
  loggingPtrs[12] = (NULL);
  loggingPtrs[13] = (NULL);
  loggingPtrs[14] = (NULL);
  loggingPtrs[15] = (NULL);
  loggingPtrs[16] = (NULL);
  loggingPtrs[17] = (NULL);
  loggingPtrs[18] = (NULL);
  loggingPtrs[19] = (NULL);
  loggingPtrs[20] = (NULL);
  loggingPtrs[21] = (NULL);
  loggingPtrs[22] = (NULL);
  loggingPtrs[23] = (NULL);
  loggingPtrs[24] = (NULL);
  loggingPtrs[25] = (NULL);
  loggingPtrs[26] = (NULL);
  loggingPtrs[27] = (NULL);
  loggingPtrs[28] = (NULL);
  loggingPtrs[29] = (NULL);
  loggingPtrs[30] = (NULL);
  loggingPtrs[31] = (NULL);
  loggingPtrs[32] = (NULL);
  loggingPtrs[33] = (NULL);
  loggingPtrs[34] = (NULL);
  loggingPtrs[35] = (NULL);
  loggingPtrs[36] = (NULL);
  loggingPtrs[37] = (NULL);
  loggingPtrs[38] = (NULL);
  loggingPtrs[39] = (NULL);
  loggingPtrs[40] = (NULL);
  loggingPtrs[41] = (NULL);
  loggingPtrs[42] = (NULL);
  loggingPtrs[43] = (NULL);
  loggingPtrs[44] = (NULL);
  loggingPtrs[45] = (NULL);
  loggingPtrs[46] = (NULL);
  loggingPtrs[47] = (NULL);
  loggingPtrs[48] = (NULL);
  loggingPtrs[49] = (NULL);
  loggingPtrs[50] = (NULL);
  loggingPtrs[51] = (NULL);
  loggingPtrs[52] = (NULL);
  loggingPtrs[53] = (NULL);
  loggingPtrs[54] = (NULL);
  loggingPtrs[55] = (NULL);
  loggingPtrs[56] = (NULL);
  loggingPtrs[57] = (NULL);
  loggingPtrs[58] = (NULL);
  loggingPtrs[59] = (NULL);
  loggingPtrs[60] = (NULL);
  loggingPtrs[61] = (NULL);
  loggingPtrs[62] = (NULL);
  loggingPtrs[63] = (NULL);
  loggingPtrs[64] = (NULL);
  loggingPtrs[65] = (NULL);
  loggingPtrs[66] = (NULL);
  loggingPtrs[67] = (NULL);
  loggingPtrs[68] = (NULL);
  loggingPtrs[69] = (NULL);
  loggingPtrs[70] = (NULL);
  loggingPtrs[71] = (NULL);
  loggingPtrs[72] = (NULL);
  loggingPtrs[73] = (NULL);
  loggingPtrs[74] = (NULL);
}

#endif

/* Data Type Map - use dataTypeMapIndex to access this structure */
static TARGET_CONST rtwCAPI_DataTypeMap rtDataTypeMap[] = {
  /* cName, mwName, numElements, elemMapIndex, dataSize, slDataId, *
   * isComplex, isPointer, enumStorageType */
  { "double", "real_T", 0, 0, sizeof(real_T), SS_DOUBLE, 0, 0, 0 }
};

#ifdef HOST_CAPI_BUILD
#undef sizeof
#endif

/* Structure Element Map - use elemMapIndex to access this structure */
static TARGET_CONST rtwCAPI_ElementMap rtElementMap[] = {
  /* elementName, elementOffset, dataTypeIndex, dimIndex, fxpIndex */
  { (NULL), 0, 0, 0, 0 },
};

/* Dimension Map - use dimensionMapIndex to access elements of ths structure*/
static rtwCAPI_DimensionMap rtDimensionMap[] = {
  /* dataOrientation, dimArrayIndex, numDims, vardimsIndex */
  { rtwCAPI_SCALAR, 0, 2, 0 },

  { rtwCAPI_VECTOR, 2, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static uint_T rtDimensionArray[] = {
  1,                                   /* 0 */
  1,                                   /* 1 */
  2,                                   /* 2 */
  1                                    /* 3 */
};

/* C-API stores floating point values in an array. The elements of this  *
 * are unique. This ensures that values which are shared across the model*
 * are stored in the most efficient way. These values are referenced by  *
 *           - rtwCAPI_FixPtMap.fracSlopePtr,                            *
 *           - rtwCAPI_FixPtMap.biasPtr,                                 *
 *           - rtwCAPI_SampleTimeMap.samplePeriodPtr,                    *
 *           - rtwCAPI_SampleTimeMap.sampleOffsetPtr                     */
static const real_T rtcapiStoredFloats[] = {
  0.001, 0.0
};

/* Fixed Point Map */
static rtwCAPI_FixPtMap rtFixPtMap[] = {
  /* fracSlopePtr, biasPtr, scaleType, wordLength, exponent, isSigned */
  { (NULL), (NULL), rtwCAPI_FIX_RESERVED, 0, 0, 0 },
};

/* Sample Time Map - use sTimeIndex to access elements of ths structure */
static rtwCAPI_SampleTimeMap rtSampleTimeMap[] = {
  /* samplePeriodPtr, sampleOffsetPtr, tid, samplingMode */
  { (const void *) &rtcapiStoredFloats[0], (const void *) &rtcapiStoredFloats[1],
    1, 0 },

  { (const void *) &rtcapiStoredFloats[1], (const void *) &rtcapiStoredFloats[1],
    0, 0 }
};

static rtwCAPI_ModelMappingStaticInfo mmiStatic = {
  /* Signals:{signals, numSignals,
   *           rootInputs, numRootInputs,
   *           rootOutputs, numRootOutputs},
   * Params: {blockParameters, numBlockParameters,
   *          modelParameters, numModelParameters},
   * States: {states, numStates},
   * Maps:   {dataTypeMap, dimensionMap, fixPtMap,
   *          elementMap, sampleTimeMap, dimensionArray},
   * TargetType: targetType
   */
  { rtBlockSignals, 28,
    (NULL), 0,
    (NULL), 0 },

  { rtBlockParameters, 27,
    rtModelParameters, 13 },

  { rtBlockStates, 7 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 1819177320U,
    746615559U,
    443682247U,
    440538563U },
  (NULL), 0,
  0
};

/* Function to get C API Model Mapping Static Info */
const rtwCAPI_ModelMappingStaticInfo*
  mdl_MoCo_Brake_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

/* Cache pointers into DataMapInfo substructure of RTModel */
#ifndef HOST_CAPI_BUILD

void mdl_MoCo_Brake_InitializeDataMapInfo(RT_MODEL_mdl_MoCo_Brake_T *const
  mdl_MoCo_Brake_M, B_mdl_MoCo_Brake_c_T *localB, X_mdl_MoCo_Brake_n_T *localX)
{
  /* Set C-API version */
  rtwCAPI_SetVersion(mdl_MoCo_Brake_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(mdl_MoCo_Brake_M->DataMapInfo.mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(mdl_MoCo_Brake_M->DataMapInfo.mmi, (NULL));

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  mdl_MoCo_Brake_InitializeDataAddr(mdl_MoCo_Brake_M->DataMapInfo.dataAddress,
    localB, localX);
  rtwCAPI_SetDataAddressMap(mdl_MoCo_Brake_M->DataMapInfo.mmi,
    mdl_MoCo_Brake_M->DataMapInfo.dataAddress);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  mdl_MoCo_Brake_InitializeVarDimsAddr
    (mdl_MoCo_Brake_M->DataMapInfo.vardimsAddress);
  rtwCAPI_SetVarDimsAddressMap(mdl_MoCo_Brake_M->DataMapInfo.mmi,
    mdl_MoCo_Brake_M->DataMapInfo.vardimsAddress);

  /* Set Instance specific path */
  rtwCAPI_SetPath(mdl_MoCo_Brake_M->DataMapInfo.mmi, (NULL));
  rtwCAPI_SetFullPath(mdl_MoCo_Brake_M->DataMapInfo.mmi, (NULL));

  /* Cache C-API logging function pointers into the Real-Time Model Data structure */
  mdl_MoCo_Brake_InitializeLoggingFunctions
    (mdl_MoCo_Brake_M->DataMapInfo.loggingPtrs);
  rtwCAPI_SetLoggingPtrs(mdl_MoCo_Brake_M->DataMapInfo.mmi,
    mdl_MoCo_Brake_M->DataMapInfo.loggingPtrs);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(mdl_MoCo_Brake_M->DataMapInfo.mmi, (NULL));

  /* Set reference to submodels */
  rtwCAPI_SetChildMMIArray(mdl_MoCo_Brake_M->DataMapInfo.mmi, (NULL));
  rtwCAPI_SetChildMMIArrayLen(mdl_MoCo_Brake_M->DataMapInfo.mmi, 0);
}

#else                                  /* HOST_CAPI_BUILD */
#ifdef __cplusplus

extern "C" {

#endif

  void mdl_MoCo_Brake_host_InitializeDataMapInfo
    (mdl_MoCo_Brake_host_DataMapInfo_T *dataMap, const char *path)
  {
    /* Set C-API version */
    rtwCAPI_SetVersion(dataMap->mmi, 1);

    /* Cache static C-API data into the Real-time Model Data structure */
    rtwCAPI_SetStaticMap(dataMap->mmi, &mmiStatic);

    /* host data address map is NULL */
    rtwCAPI_SetDataAddressMap(dataMap->mmi, NULL);

    /* host vardims address map is NULL */
    rtwCAPI_SetVarDimsAddressMap(dataMap->mmi, NULL);

    /* Set Instance specific path */
    rtwCAPI_SetPath(dataMap->mmi, path);
    rtwCAPI_SetFullPath(dataMap->mmi, NULL);

    /* Set reference to submodels */
    rtwCAPI_SetChildMMIArray(dataMap->mmi, (NULL));
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 0);
  }

#ifdef __cplusplus

}
#endif
#endif                                 /* HOST_CAPI_BUILD */

/* EOF: mdl_MoCo_Brake_capi.c */
