//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef AP_PSM_APP_ENGINE_REQ_C_H_
#define AP_PSM_APP_ENGINE_REQ_C_H_

#include "Platform_Types.h"
#include "ap_commonvehsigprovider/start_stop_status_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ///Stop engine
    boolean stopEngine_nu;
    ///Start engine
    boolean startEngine_nu;
    ///Status of start stop function
    AP_COMMONVEHSIGPROVIDER_StartStopStatus startStopStatus_nu;
} AP_PSM_APP_EngineReq;

inline AP_PSM_APP_EngineReq create_AP_PSM_APP_EngineReq(void)
{
  AP_PSM_APP_EngineReq m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // AP_PSM_APP_ENGINE_REQ_C_H_
