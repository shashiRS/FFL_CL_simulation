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

#ifndef AP_PSM_APP_PSMTO_SSPPORT_C_H_
#define AP_PSM_APP_PSMTO_SSPPORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "ap_psm_app/engine_req_c.h"
#include "ap_psm_app/bcmreq_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///Signals from Engine ECU
    AP_PSM_APP_EngineReq engineReq;
    ///Signals to BCM
    AP_PSM_APP_BCMReq bcmReq;
} AP_PSM_APP_PSMToSSPPort;

inline AP_PSM_APP_PSMToSSPPort create_AP_PSM_APP_PSMToSSPPort(void)
{
  AP_PSM_APP_PSMToSSPPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.engineReq = create_AP_PSM_APP_EngineReq();
  m.bcmReq = create_AP_PSM_APP_BCMReq();
  return m;
}

#endif // AP_PSM_APP_PSMTO_SSPPORT_C_H_