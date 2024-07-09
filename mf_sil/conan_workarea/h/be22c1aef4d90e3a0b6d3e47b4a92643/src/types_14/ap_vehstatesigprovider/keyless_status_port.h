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

#ifndef AP_VEHSTATESIGPROVIDER_KEYLESS_STATUS_PORT_H_
#define AP_VEHSTATESIGPROVIDER_KEYLESS_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_vehstatesigprovider/key_fob_user_action.h"
#include "Platform_Types.h"
#include "ap_vehstatesigprovider/key_fob_in_range.h"


namespace ap_vehstatesigprovider
{

  /// Signals from Keyless Go (VW: Kessy) ECU
  struct KeylessStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///User action from the key fob
    KeyFobUserAction keyFobUserAction{};
    ///@unit{nu}
    ///@range{0,15}
    ///Will be increased by one each time the key communicates with HFM ( when the key is in the range)
    uint8 keyFobButtonAliveCounter{};
    ///@range{0,1}
    ///information regarding the location of the key
    KeyFobInRange keyFobInRange{};
    ///@range{0,15}
    ///Will be increased with each can message  (has to be dropped because it will be checked and abstracted on higher level)
    uint8 keylessStatusPortCANAlive_nu{};
  };

} // namespace ap_vehstatesigprovider

#endif // AP_VEHSTATESIGPROVIDER_KEYLESS_STATUS_PORT_H_