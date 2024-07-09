#ifndef RTW_HEADER_MoCo_Brake_Autocode_cap_host_h_
#define RTW_HEADER_MoCo_Brake_Autocode_cap_host_h_
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"
#include "mdl_MoCo_Brake_capi_host.h"

typedef struct {
  rtwCAPI_ModelMappingInfo mmi;
  rtwCAPI_ModelMappingInfo *childMMI[1];
  mdl_MoCo_Brake_host_DataMapInfo_T child0;
} MoCo_Brake_Autocode_host_DataMapInfo_T;

#ifdef __cplusplus

extern "C" {

#endif

  void MoCo_Brake_Autocode_host_InitializeDataMapInfo
    (MoCo_Brake_Autocode_host_DataMapInfo_T *dataMap, const char *path);

#ifdef __cplusplus

}
#endif
#endif                                 /* HOST_CAPI_BUILD */
#endif                          /* RTW_HEADER_MoCo_Brake_Autocode_cap_host_h_ */

/* EOF: MoCo_Brake_Autocode_capi_host.h */
