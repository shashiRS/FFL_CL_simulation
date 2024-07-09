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

#ifndef AP_VEHSTATESIGPROVIDER_CHASSIS_AXLE_HEIGHT_PORT_H_
#define AP_VEHSTATESIGPROVIDER_CHASSIS_AXLE_HEIGHT_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"


namespace ap_vehstatesigprovider
{

  /// axle height information
  struct ChassisAxleHeightPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@unit{nu}
    ///Height in mm between chassis and front axle
    float32 chassisAxleHeightFront_mm{};
    ///@unit{nu}
    ///Height in mm between chassis and rear axle
    float32 chassisAxleHeightRear_mm{};
  };

} // namespace ap_vehstatesigprovider

#endif // AP_VEHSTATESIGPROVIDER_CHASSIS_AXLE_HEIGHT_PORT_H_