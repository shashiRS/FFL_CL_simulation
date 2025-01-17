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

#ifndef AP_COMMONVEHSIGPROVIDER_PSMSAMPLE_TIME_PORT_H_
#define AP_COMMONVEHSIGPROVIDER_PSMSAMPLE_TIME_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_commonvehsigprovider
{

  /// None
  struct PSMSampleTimePort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@unit{microsecond}
    ///
    uint64 psmSampleTime_us;
  };

  inline ::ap_commonvehsigprovider::PSMSampleTimePort createPSMSampleTimePort()
  {
    PSMSampleTimePort m;
    (void)::eco::memset(&m, 0U, sizeof(PSMSampleTimePort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_commonvehsigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_commonvehsigprovider::PSMSampleTimePort create_default()
  {
      return ::ap_commonvehsigprovider::createPSMSampleTimePort();
  }
}


#endif // AP_COMMONVEHSIGPROVIDER_PSMSAMPLE_TIME_PORT_H_
