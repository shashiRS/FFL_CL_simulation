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

#ifndef AP_LODMC_MANEUVERING_FINISHED_H_
#define AP_LODMC_MANEUVERING_FINISHED_H_

#include "Platform_Types.h"

namespace ap_lodmc
{
  ///Status of the maneuvering.
  enum class maneuveringFinished : uint8
  {
      MANEUVERING_NOT_STARTED = 0U,
      MANEUVERING_IN_PROGRESS = 1U,
      MANEUVERING_FINISHED = 2U,
      MANEUVERING_SATURATED = 3U,
      MANEUVERING_FAULT = 4U,
      RESERVED = 5U,
  };
} // namespace ap_lodmc
#endif // AP_LODMC_MANEUVERING_FINISHED_H_