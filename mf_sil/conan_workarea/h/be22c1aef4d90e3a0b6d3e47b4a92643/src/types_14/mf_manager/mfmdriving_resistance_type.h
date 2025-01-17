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

#ifndef MF_MANAGER_MFMDRIVING_RESISTANCE_TYPE_H_
#define MF_MANAGER_MFMDRIVING_RESISTANCE_TYPE_H_

#include "Platform_Types.h"

namespace mf_manager
{
  enum class MFMDrivingResistanceType : uint8
  {
      MFM_NONE = 0U,
      MFM_FALLING_LOW = 1U,
      MFM_FALLING_MEDIUM = 2U,
      MFM_FALLING_HIGH = 3U,
      MFM_RISING_LOW = 4U,
      MFM_RISING_MEDIUM = 5U,
      MFM_RISING_HIGH = 6U,
      MFM_WHEEL_STOPPER = 7U,
  };
} // namespace mf_manager
#endif // MF_MANAGER_MFMDRIVING_RESISTANCE_TYPE_H_
