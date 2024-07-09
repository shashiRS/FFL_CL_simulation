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

#ifndef AP_TRJCTL_MOCO_TE_MOT_REQ_H_
#define AP_TRJCTL_MOCO_TE_MOT_REQ_H_

#include "Platform_Types.h"

namespace ap_trjctl
{
  ///Motion request (MoCo variant only)
  enum class MOCO_te_MotReq : uint8
  {
      MOCO_MOT_REQ_SUPPRESS_STANDSTILL = 0U,
      MOCO_MOT_REQ_STANDSTILL = 1U,
      MOCO_MOT_REQ_DRIVEOFF = 2U,
  };
} // namespace ap_trjctl
#endif // AP_TRJCTL_MOCO_TE_MOT_REQ_H_
