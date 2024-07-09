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

#ifndef PDCP_CRITICALITY_LEVEL_H_
#define PDCP_CRITICALITY_LEVEL_H_

#include "Platform_Types.h"

namespace pdcp
{
  ///Criticality level for this sector (the higher the criticality, the closer the obstacle is to the car).
  enum class CriticalityLevel : uint8
  {
      PDC_CRIT_LVL_OUTSIDE = 0U,
      PDC_CRIT_LVL_GREEN = 1U,
      PDC_CRIT_LVL_YELLOW = 2U,
      PDC_CRIT_LVL_RED = 3U,
  };
} // namespace pdcp
#endif // PDCP_CRITICALITY_LEVEL_H_