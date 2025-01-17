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

#ifndef AP_TRJCTL_MF_CONTROL_TE_LONG_CTRL_MODE_H_
#define AP_TRJCTL_MF_CONTROL_TE_LONG_CTRL_MODE_H_

#include "Platform_Types.h"

namespace ap_trjctl
{
  ///Type of the longitudinal control mode (MoCo variant only)
  enum class MF_CONTROL_te_LongCtrlMode : uint8
  {
      MF_LONG_CTRL_MODE_NO_CONTROL = 0U,
      MF_LONG_CTRL_MODE_DIST_VELO_MAX = 1U,
      MF_LONG_CTRL_MODE_ACCEL = 2U,
      MF_LONG_CTRL_MODE_VELO_LIMITATION = 3U,
  };
} // namespace ap_trjctl
#endif // AP_TRJCTL_MF_CONTROL_TE_LONG_CTRL_MODE_H_
