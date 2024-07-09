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

#ifndef AP_COMMONVEHSIGPROVIDER_GEAR_H_
#define AP_COMMONVEHSIGPROVIDER_GEAR_H_

#include "Platform_Types.h"

namespace ap_commonvehsigprovider
{
  ///System status/Availability of the gearbox control and current gear.
  enum class Gear : uint8
  {
      GEAR_N = 0U,
      GEAR_1 = 1U,
      GEAR_2 = 2U,
      GEAR_3 = 3U,
      GEAR_4 = 4U,
      GEAR_5 = 5U,
      GEAR_6 = 6U,
      GEAR_7 = 7U,
      GEAR_8 = 8U,
      GEAR_P = 9U,
      GEAR_S = 10U,
      GEAR_D = 11U,
      GEAR_INTERMEDIATE_POS = 12U,
      GEAR_R = 13U,
      GEAR_NOT_DEFINED = 14U,
      GEAR_ERROR = 15U,
  };
} // namespace ap_commonvehsigprovider
#endif // AP_COMMONVEHSIGPROVIDER_GEAR_H_