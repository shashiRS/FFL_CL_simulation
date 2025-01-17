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

#ifndef AP_PSM_DRIVING_MODE_TRC_H_
#define AP_PSM_DRIVING_MODE_TRC_H_

#include "Platform_Types.h"

namespace ap_psm
{
  ///Define the requested driving mode. Only in the mode "FOLLOW_TRAJ" the requested trajectory will be used as input for the trajectory control.
  enum class DrivingModeTRC : uint8
  {
      NO_INTERVENTION = 0U,
      FOLLOW_TRAJ = 1U,
      MAKE_PAUSE = 2U,
      MAKE_SECURE = 3U,
      MAKE_SECURE_AND_ADJUST_ANGLE = 4U,
  };
} // namespace ap_psm
#endif // AP_PSM_DRIVING_MODE_TRC_H_
