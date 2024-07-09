// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef AP_PSM_APP_PSMTO_SSPPORT_H_
#define AP_PSM_APP_PSMTO_SSPPORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_psm_app/engine_req.h"
#include "ap_psm_app/bcmreq.h"


namespace ap_psm_app
{

  struct PSMToSSPPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///Signals from Engine ECU
    EngineReq engineReq{};
    ///Signals to BCM
    BCMReq bcmReq{};
  };

} // namespace ap_psm_app

#endif // AP_PSM_APP_PSMTO_SSPPORT_H_