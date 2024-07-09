// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef AP_PSM_APP_PSMTO_SSPPORT_H_
#define AP_PSM_APP_PSMTO_SSPPORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_psm_app/engine_req.h"
#include "ap_psm_app/bcmreq.h"
#include "eco/memset.h"


namespace ap_psm_app
{

  struct PSMToSSPPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Signals from Engine ECU
    EngineReq engineReq;
    ///Signals to BCM
    BCMReq bcmReq;
  };

  inline ::ap_psm_app::PSMToSSPPort createPSMToSSPPort()
  {
    PSMToSSPPort m;
    (void)::eco::memset(&m, 0U, sizeof(PSMToSSPPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.engineReq = createEngineReq();
    m.bcmReq = createBCMReq();
    return m;
  }

} // namespace ap_psm_app

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_psm_app::PSMToSSPPort create_default()
  {
      return ::ap_psm_app::createPSMToSSPPort();
  }
}


#endif // AP_PSM_APP_PSMTO_SSPPORT_H_