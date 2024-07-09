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

#ifndef AP_TP_TARGET_SIDE_H_
#define AP_TP_TARGET_SIDE_H_

#include "Platform_Types.h"

namespace ap_tp
{
  ///
  enum class TargetSide : uint8
  {
      TS_RIGHT_SIDE = 0U,
      TS_LEFT_SIDE = 1U,
      TS_IN_FRONT_RIGHT = 2U,
      TS_IN_FRONT_CENTER = 3U,
      TS_IN_FRONT_LEFT = 4U,
      TS_IN_REAR_RIGHT = 5U,
      TS_IN_REAR_CENTER = 6U,
      TS_IN_REAR_LEFT = 7U,
      TS_UNDEFINED_SIDE = 8U,
  };
} // namespace ap_tp
#endif // AP_TP_TARGET_SIDE_H_
