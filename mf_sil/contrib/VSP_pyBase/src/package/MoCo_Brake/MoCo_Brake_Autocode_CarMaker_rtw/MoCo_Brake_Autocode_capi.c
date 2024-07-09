/*
 * MoCo_Brake_Autocode_capi.c
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

#include "rtw_capi.h"
#ifdef HOST_CAPI_BUILD
#include "MoCo_Brake_Autocode_capi_host.h"
#define sizeof(s)                      ((size_t)(0xFFFF))
#undef rt_offsetof
#define rt_offsetof(s,el)              ((uint16_T)(0xFFFF))
#define TARGET_CONST
#define TARGET_STRING(s)               (s)
#else                                  /* HOST_CAPI_BUILD */
#include "builtin_typeid_types.h"
#include "MoCo_Brake_Autocode.h"
#include "MoCo_Brake_Autocode_capi.h"
#include "MoCo_Brake_Autocode_private.h"
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
  { 0, 0, TARGET_STRING("MoCo_Brake_Autocode/brake_torque_defined_positive"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 1, 0, TARGET_STRING("MoCo_Brake_Autocode/brake_torque_defined_positive1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 2, 0, TARGET_STRING("MoCo_Brake_Autocode/Model"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 3, 0, TARGET_STRING("MoCo_Brake_Autocode/Model"),
    TARGET_STRING(""), 1, 0, 0, 0, 0 },

  { 4, 0, TARGET_STRING("MoCo_Brake_Autocode/Model"),
    TARGET_STRING(""), 2, 0, 0, 0, 0 },

  { 5, 0, TARGET_STRING("MoCo_Brake_Autocode/Model"),
    TARGET_STRING(""), 3, 0, 0, 0, 0 },

  { 6, 0, TARGET_STRING("MoCo_Brake_Autocode/Multiply"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 7, 0, TARGET_STRING("MoCo_Brake_Autocode/Multiply1"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 8, 0, TARGET_STRING("MoCo_Brake_Autocode/Multiply2"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 9, 0, TARGET_STRING("MoCo_Brake_Autocode/Multiply3"),
    TARGET_STRING(""), 0, 0, 0, 0, 0 },

  { 10, 0, TARGET_STRING("MoCo_Brake_Autocode/Read CM Dict"),
    TARGET_STRING("SET_TRQ"), 0, 0, 0, 0, 0 },

  { 11, 0, TARGET_STRING("MoCo_Brake_Autocode/Read CM Dict1"),
    TARGET_STRING("HOLD_REQ"), 0, 0, 0, 0, 0 },

  { 12, 0, TARGET_STRING("MoCo_Brake_Autocode/Read CM Dict2"),
    TARGET_STRING("GO_REQ"), 0, 0, 0, 0, 0 },

  {
    0, 0, (NULL), (NULL), 0, 0, 0, 0, 0
  }
};

static rtwCAPI_BlockParameters rtBlockParameters[] = {
  /* addrMapIndex, blockPath,
   * paramName, dataTypeIndex, dimIndex, fixPtIdx
   */
  { 13, TARGET_STRING("MoCo_Brake_Autocode/brake_torque_defined_positive"),
    TARGET_STRING("Gain"), 0, 0, 0 },

  { 14, TARGET_STRING("MoCo_Brake_Autocode/brake_torque_defined_positive1"),
    TARGET_STRING("Gain"), 0, 0, 0 },

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
  {
    0, -1, (NULL), (NULL), (NULL), 0, 0, 0, 0, 0, 0, -1, 0
  }
};

/* Tunable variable parameters */
static rtwCAPI_ModelParameters rtModelParameters[] = {
  /* addrMapIndex, varName, dataTypeIndex, dimIndex, fixPtIndex */
  { 15, TARGET_STRING("Brk_SSM_Decel_Lim"), 0, 0, 0 },

  { 16, TARGET_STRING("D_brk"), 0, 0, 0 },

  { 17, TARGET_STRING("K_brk"), 0, 0, 0 },

  { 18, TARGET_STRING("K_brk_accel_dec"), 0, 0, 0 },

  { 19, TARGET_STRING("K_brk_trq_dec"), 0, 0, 0 },

  { 20, TARGET_STRING("T_brk_accel_dec"), 0, 0, 0 },

  { 21, TARGET_STRING("T_brk_trq_dec"), 0, 0, 0 },

  { 22, TARGET_STRING("Trq_Distrib_FL"), 0, 0, 0 },

  { 23, TARGET_STRING("Trq_Distrib_FR"), 0, 0, 0 },

  { 24, TARGET_STRING("Trq_Distrib_RL"), 0, 0, 0 },

  { 25, TARGET_STRING("Trq_Distrib_RR"), 0, 0, 0 },

  { 26, TARGET_STRING("accel_to_trq"), 0, 0, 0 },

  { 27, TARGET_STRING("brake_pres_exist_threshold"), 0, 0, 0 },

  { 28, TARGET_STRING("delay_brake_inc"), 0, 0, 0 },

  { 29, TARGET_STRING("delay_brake_inc_init"), 0, 0, 0 },

  { 30, TARGET_STRING("w0_brk"), 0, 0, 0 },

  { 31, TARGET_STRING("w0_brkinit"), 0, 0, 0 },

  { 0, (NULL), 0, 0, 0 }
};

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
  { rtwCAPI_SCALAR, 0, 2, 0 }
};

/* Dimension Array- use dimArrayIndex to access elements of this array */
static uint_T rtDimensionArray[] = {
  1,                                   /* 0 */
  1                                    /* 1 */
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
    1, 0 }
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
  { rtBlockSignals, 13,
    (NULL), 0,
    (NULL), 0 },

  { rtBlockParameters, 2,
    rtModelParameters, 17 },

  { rtBlockStates, 0 },

  { rtDataTypeMap, rtDimensionMap, rtFixPtMap,
    rtElementMap, rtSampleTimeMap, rtDimensionArray },
  "float",

  { 273559912U,
    1589008871U,
    909737391U,
    1905268448U },
  (NULL), 0,
  0
};

/* Function to get C API Model Mapping Static Info */
const rtwCAPI_ModelMappingStaticInfo*
  MoCo_Brake_Autocode_GetCAPIStaticMap(void)
{
  return &mmiStatic;
}

/* Cache pointers into DataMapInfo substructure of RTModel */
#ifndef HOST_CAPI_BUILD

void MoCo_Brake_Autocode_InitializeDataMapInfo(RT_MODEL_MoCo_Brake_Autocode_T *
  const MoCo_Brake_Autocode_M)
{
  /* run-time setup of addresses */
  B_MoCo_Brake_Autocode_T *MoCo_Brake_Autocode_B = (B_MoCo_Brake_Autocode_T *)
    MoCo_Brake_Autocode_M->blockIO;
  void* *rtDataAddrMap;
  int32_T* *rtVarDimsAddrMap;
  rt_FREE( rtwCAPI_GetDataAddressMap( &(MoCo_Brake_Autocode_M->DataMapInfo.mmi) )
          );
  rtDataAddrMap = (void* *) malloc(32 * sizeof(void* ));
  if ((rtDataAddrMap) == (NULL)) {
    rtmSetErrorStatus(MoCo_Brake_Autocode_M, RT_MEMORY_ALLOCATION_ERROR);
    return;
  }

  rtDataAddrMap[0] = (void* )
    (&MoCo_Brake_Autocode_B->brake_torque_defined_positive);
  rtDataAddrMap[1] = (void* )
    (&MoCo_Brake_Autocode_B->brake_torque_defined_positive1);
  rtDataAddrMap[2] = (void* )(&MoCo_Brake_Autocode_B->Model_o1);
  rtDataAddrMap[3] = (void* )(&MoCo_Brake_Autocode_B->Model_o2);
  rtDataAddrMap[4] = (void* )(&MoCo_Brake_Autocode_B->Model_o3);
  rtDataAddrMap[5] = (void* )(&MoCo_Brake_Autocode_B->Model_o4);
  rtDataAddrMap[6] = (void* )(&MoCo_Brake_Autocode_B->Multiply);
  rtDataAddrMap[7] = (void* )(&MoCo_Brake_Autocode_B->Multiply1);
  rtDataAddrMap[8] = (void* )(&MoCo_Brake_Autocode_B->Multiply2);
  rtDataAddrMap[9] = (void* )(&MoCo_Brake_Autocode_B->Multiply3);
  rtDataAddrMap[10] = (void* )(&MoCo_Brake_Autocode_B->SET_TRQ);
  rtDataAddrMap[11] = (void* )(&MoCo_Brake_Autocode_B->HOLD_REQ);
  rtDataAddrMap[12] = (void* )(&MoCo_Brake_Autocode_B->GO_REQ);
  rtDataAddrMap[13] = (void* )
    (&MoCo_Brake_Autocode_P.brake_torque_defined_positive_Gain);
  rtDataAddrMap[14] = (void* )
    (&MoCo_Brake_Autocode_P.brake_torque_defined_positive1_Gain);
  rtDataAddrMap[15] = (void* )(&rtP_Brk_SSM_Decel_Lim);
  rtDataAddrMap[16] = (void* )(&rtP_D_brk);
  rtDataAddrMap[17] = (void* )(&rtP_K_brk);
  rtDataAddrMap[18] = (void* )(&rtP_K_brk_accel_dec);
  rtDataAddrMap[19] = (void* )(&rtP_K_brk_trq_dec);
  rtDataAddrMap[20] = (void* )(&rtP_T_brk_accel_dec);
  rtDataAddrMap[21] = (void* )(&rtP_T_brk_trq_dec);
  rtDataAddrMap[22] = (void* )(&rtP_Trq_Distrib_FL);
  rtDataAddrMap[23] = (void* )(&rtP_Trq_Distrib_FR);
  rtDataAddrMap[24] = (void* )(&rtP_Trq_Distrib_RL);
  rtDataAddrMap[25] = (void* )(&rtP_Trq_Distrib_RR);
  rtDataAddrMap[26] = (void* )(&rtP_accel_to_trq);
  rtDataAddrMap[27] = (void* )(&rtP_brake_pres_exist_threshold);
  rtDataAddrMap[28] = (void* )(&rtP_delay_brake_inc);
  rtDataAddrMap[29] = (void* )(&rtP_delay_brake_inc_init);
  rtDataAddrMap[30] = (void* )(&rtP_w0_brk);
  rtDataAddrMap[31] = (void* )(&rtP_w0_brkinit);
  rt_FREE( rtwCAPI_GetVarDimsAddressMap
          ( &(MoCo_Brake_Autocode_M->DataMapInfo.mmi) ) );
  rtVarDimsAddrMap = (int32_T* *) malloc(1 * sizeof(int32_T* ));
  if ((rtVarDimsAddrMap) == (NULL)) {
    rtmSetErrorStatus(MoCo_Brake_Autocode_M, RT_MEMORY_ALLOCATION_ERROR);
    return;
  }

  rtVarDimsAddrMap[0] = (int32_T* )((NULL));

  /* Set C-API version */
  rtwCAPI_SetVersion(MoCo_Brake_Autocode_M->DataMapInfo.mmi, 1);

  /* Cache static C-API data into the Real-time Model Data structure */
  rtwCAPI_SetStaticMap(MoCo_Brake_Autocode_M->DataMapInfo.mmi, &mmiStatic);

  /* Cache static C-API logging data into the Real-time Model Data structure */
  rtwCAPI_SetLoggingStaticMap(MoCo_Brake_Autocode_M->DataMapInfo.mmi, (NULL));

  /* Cache C-API Data Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetDataAddressMap(MoCo_Brake_Autocode_M->DataMapInfo.mmi,
    rtDataAddrMap);

  /* Cache C-API Data Run-Time Dimension Buffer Addresses into the Real-Time Model Data structure */
  rtwCAPI_SetVarDimsAddressMap(MoCo_Brake_Autocode_M->DataMapInfo.mmi,
    rtVarDimsAddrMap);

  /* Cache the instance C-API logging pointer */
  rtwCAPI_SetInstanceLoggingInfo(MoCo_Brake_Autocode_M->DataMapInfo.mmi, (NULL));

  /* Set reference to submodels */
  rtwCAPI_SetChildMMIArray(MoCo_Brake_Autocode_M->DataMapInfo.mmi,
    MoCo_Brake_Autocode_M->DataMapInfo.childMMI);
  rtwCAPI_SetChildMMIArrayLen(MoCo_Brake_Autocode_M->DataMapInfo.mmi, 1);
}

#else                                  /* HOST_CAPI_BUILD */
#ifdef __cplusplus

extern "C" {

#endif

  void MoCo_Brake_Autocode_host_InitializeDataMapInfo
    (MoCo_Brake_Autocode_host_DataMapInfo_T *dataMap, const char *path)
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
    dataMap->childMMI[0] = &(dataMap->child0.mmi);
    mdl_MoCo_Brake_host_InitializeDataMapInfo(&(dataMap->child0),
      "MoCo_Brake_Autocode/Model");
    rtwCAPI_SetChildMMIArray(dataMap->mmi, dataMap->childMMI);
    rtwCAPI_SetChildMMIArrayLen(dataMap->mmi, 1);
  }

#ifdef __cplusplus

}
#endif
#endif                                 /* HOST_CAPI_BUILD */

/* EOF: MoCo_Brake_Autocode_capi.c */
