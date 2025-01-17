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

#ifndef AP_TP_VLPOSITIONING_APPROACH_H_
#define AP_TP_VLPOSITIONING_APPROACH_H_

#include "Platform_Types.h"

namespace ap_tp
{
  ///
  enum class VLPositioningApproach : uint8
  {
      ALIGN_TO_FRONT_VL_PT = 0U,
      ALIGN_TO_REAR_VL_PT = 1U,
      AVG_BOTH_VL_PTS_NEAR_SLOT = 2U,
  };
} // namespace ap_tp
#endif // AP_TP_VLPOSITIONING_APPROACH_H_
