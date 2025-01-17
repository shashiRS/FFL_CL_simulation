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

#ifndef AP_VEHSTATESIGPROVIDER_ALL_DOOR_STATUS_H_
#define AP_VEHSTATESIGPROVIDER_ALL_DOOR_STATUS_H_

#include "ap_vehstatesigprovider/door_status.h"


namespace ap_vehstatesigprovider
{

  /// Status of all doors
  struct AllDoorStatus
  {
    ///@range{0,3}
    ///Door of the front passenger is open
    DoorStatus frontPsgr_nu{};
    ///@range{0,3}
    ///Door of the driver is open
    DoorStatus driver_nu{};
    ///@range{0,3}
    ///Rear door on the right (behind front passenger) open
    DoorStatus rearRight_nu{};
    ///@range{0,3}
    ///Rear door on the left (behind driver) open
    DoorStatus rearLeft_nu{};
  };

} // namespace ap_vehstatesigprovider

#endif // AP_VEHSTATESIGPROVIDER_ALL_DOOR_STATUS_H_
