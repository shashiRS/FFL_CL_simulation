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

#ifndef AP_COMMONVEHSIGPROVIDER_WHEEL_DRIVING_DIRECTION_H_
#define AP_COMMONVEHSIGPROVIDER_WHEEL_DRIVING_DIRECTION_H_

#include "Platform_Types.h"

namespace ap_commonvehsigprovider
{
  ///rotational direction wheel front left
  enum class WheelDrivingDirection : sint8
  {
      WHEEL_DIRECTION_REVERSE = -1,
      WHEEL_DIRECTION_INIT_INVALID = 0,
      WHEEL_DIRECTION_FORWARD = 1,
  };
} // namespace ap_commonvehsigprovider
#endif // AP_COMMONVEHSIGPROVIDER_WHEEL_DRIVING_DIRECTION_H_