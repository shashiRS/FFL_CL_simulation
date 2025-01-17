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

#ifndef AP_PSM_APPLANNING_SPECIFICATION_H_
#define AP_PSM_APPLANNING_SPECIFICATION_H_

#include "Platform_Types.h"

namespace ap_psm
{
  ///Additional information to consider for the planning of the requested parking maneuver.
  enum class APPlanningSpecification : uint8
  {
      APPS_INVALID = 0U,
      APPS_NONE = 1U,
      APPS_PARK_IN_FULL_MANEUVERING_AREA = 2U,
      APPS_PARK_IN_RESTRICTED_MANEUVERING_AREA = 3U,
      APPS_PARK_OUT_UNTIL_CRITICAL_POINT_REACHED = 4U,
      APPS_PARK_OUT_TO_TARGET_POSE = 5U,
  };
} // namespace ap_psm
#endif // AP_PSM_APPLANNING_SPECIFICATION_H_
