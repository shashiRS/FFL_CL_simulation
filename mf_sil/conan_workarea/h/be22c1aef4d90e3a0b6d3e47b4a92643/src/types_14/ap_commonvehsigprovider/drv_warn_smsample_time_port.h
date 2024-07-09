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

#ifndef AP_COMMONVEHSIGPROVIDER_DRV_WARN_SMSAMPLE_TIME_PORT_H_
#define AP_COMMONVEHSIGPROVIDER_DRV_WARN_SMSAMPLE_TIME_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"


namespace ap_commonvehsigprovider
{

  /// None
  struct DrvWarnSMSampleTimePort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@unit{microsecond}
    ///Sample time between last and current call of  DrvWarnSM function component
    uint64 drvWarnSMSampleTime_us{};
  };

} // namespace ap_commonvehsigprovider

#endif // AP_COMMONVEHSIGPROVIDER_DRV_WARN_SMSAMPLE_TIME_PORT_H_
