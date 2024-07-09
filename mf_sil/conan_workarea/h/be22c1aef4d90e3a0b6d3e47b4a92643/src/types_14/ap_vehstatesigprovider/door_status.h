// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

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

#ifndef AP_VEHSTATESIGPROVIDER_DOOR_STATUS_H_
#define AP_VEHSTATESIGPROVIDER_DOOR_STATUS_H_

#include "Platform_Types.h"

namespace ap_vehstatesigprovider
{
  ///Door of the front passenger is open
  enum class DoorStatus : uint8
  {
      DOOR_STATUS_INIT = 0U,
      DOOR_STATUS_OPEN = 1U,
      DOOR_STATUS_CLOSED = 2U,
      DOOR_STATUS_LOCKED = 3U,
      DOOR_STATUS_ERROR = 4U,
  };
} // namespace ap_vehstatesigprovider
#endif // AP_VEHSTATESIGPROVIDER_DOOR_STATUS_H_
