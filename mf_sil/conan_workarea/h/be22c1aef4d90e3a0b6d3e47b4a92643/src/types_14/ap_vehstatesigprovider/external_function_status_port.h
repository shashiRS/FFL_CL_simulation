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

#ifndef AP_VEHSTATESIGPROVIDER_EXTERNAL_FUNCTION_STATUS_PORT_H_
#define AP_VEHSTATESIGPROVIDER_EXTERNAL_FUNCTION_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_vehstatesigprovider/rctastatus.h"
#include "ap_vehstatesigprovider/ebastatus.h"


namespace ap_vehstatesigprovider
{

  /// Status of other functions that have special influence on AP
  struct ExternalFunctionStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@range{0,3}
    ///Rear cross traffic alert status
    RCTAStatus rctaStatus_nu{};
    ///Status of the emergency brake assist
    EBAStatus ebaStatus_nu{};
  };

} // namespace ap_vehstatesigprovider

#endif // AP_VEHSTATESIGPROVIDER_EXTERNAL_FUNCTION_STATUS_PORT_H_
