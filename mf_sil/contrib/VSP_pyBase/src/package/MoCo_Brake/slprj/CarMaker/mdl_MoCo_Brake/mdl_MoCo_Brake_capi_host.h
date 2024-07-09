#ifndef RTW_HEADER_mdl_MoCo_Brake_cap_host_h_
#define RTW_HEADER_mdl_MoCo_Brake_cap_host_h_
#ifdef HOST_CAPI_BUILD
#include "rtw_capi.h"
#include "rtw_modelmap.h"

typedef struct {
  rtwCAPI_ModelMappingInfo mmi;
} mdl_MoCo_Brake_host_DataMapInfo_T;

#ifdef __cplusplus

extern "C" {

#endif

  void mdl_MoCo_Brake_host_InitializeDataMapInfo
    (mdl_MoCo_Brake_host_DataMapInfo_T *dataMap, const char *path);

#ifdef __cplusplus

}
#endif
#endif                                 /* HOST_CAPI_BUILD */
#endif                               /* RTW_HEADER_mdl_MoCo_Brake_cap_host_h_ */

/* EOF: mdl_MoCo_Brake_capi_host.h */
