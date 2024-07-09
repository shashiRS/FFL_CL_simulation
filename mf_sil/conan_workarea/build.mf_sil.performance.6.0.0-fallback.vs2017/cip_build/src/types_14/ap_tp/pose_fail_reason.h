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

#ifndef AP_TP_POSE_FAIL_REASON_H_
#define AP_TP_POSE_FAIL_REASON_H_

#include "Platform_Types.h"

namespace ap_tp
{
  ///
  enum class PoseFailReason : uint8
  {
      TAPOSD_PFR_NONE = 0U,
      TAPOSD_PFR_PARKING_BOX_WIDTH_TOO_NARROW = 1U,
      TAPOSD_PFR_PARKING_BOX_LENGTH_TOO_SHORT = 2U,
      TAPOSD_PFR_MAXBOX_EXCEEDED = 3U,
      TAPOSD_PFR_WHEEL_COLLISION = 4U,
      TAPOSD_PFR_HIGH_OBJECT_COLLISION = 5U,
      TAPOSD_PFR_UNKNOWN = 6U,
      MAX_NUM_POSE_FAIL_TYPES = 7U,
  };
} // namespace ap_tp
#endif // AP_TP_POSE_FAIL_REASON_H_
