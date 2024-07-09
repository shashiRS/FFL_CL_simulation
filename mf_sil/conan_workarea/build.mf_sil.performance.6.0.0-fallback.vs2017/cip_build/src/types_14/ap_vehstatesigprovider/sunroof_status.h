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

#ifndef AP_VEHSTATESIGPROVIDER_SUNROOF_STATUS_H_
#define AP_VEHSTATESIGPROVIDER_SUNROOF_STATUS_H_

#include "Platform_Types.h"

namespace ap_vehstatesigprovider
{
  ///Status of vehicle sunroof
  enum class SunroofStatus : uint8
  {
      SUNROOF_STATUS_CLOSED = 0U,
      SUNROOF_STATUS_OPEN = 1U,
      SUNROOF_STATUS_OPENING = 2U,
      SUNROOF_STATUS_CLOSING = 3U,
      SUNROOF_STATUS_INVALID = 4U,
  };
} // namespace ap_vehstatesigprovider
#endif // AP_VEHSTATESIGPROVIDER_SUNROOF_STATUS_H_
