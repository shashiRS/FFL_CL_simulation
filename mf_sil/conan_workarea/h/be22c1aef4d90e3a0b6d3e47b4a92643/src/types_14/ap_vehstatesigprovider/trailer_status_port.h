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

#ifndef AP_VEHSTATESIGPROVIDER_TRAILER_STATUS_PORT_H_
#define AP_VEHSTATESIGPROVIDER_TRAILER_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_vehstatesigprovider/trailer_hitch_status.h"


namespace ap_vehstatesigprovider
{

  /// Signals from Trailer ECU
  struct TrailerStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///Attached trailer recognized
    boolean trailerAttached_nu{};
    ///@range{0,5}
    ///Status of trailer hitch
    TrailerHitchStatus trailerHitchStatus_nu{};
  };

} // namespace ap_vehstatesigprovider

#endif // AP_VEHSTATESIGPROVIDER_TRAILER_STATUS_PORT_H_
