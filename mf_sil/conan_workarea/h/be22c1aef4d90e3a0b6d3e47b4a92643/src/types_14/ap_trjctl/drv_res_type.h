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

#ifndef AP_TRJCTL_DRV_RES_TYPE_H_
#define AP_TRJCTL_DRV_RES_TYPE_H_

#include "Platform_Types.h"

namespace ap_trjctl
{
  ///Type of the driving resistance. (e.g. for crossing curbstone or approaching wheel stopper)
  enum class DrvResType : uint8
  {
      DRVRES_NONE = 0U,
      DRVRES_FALLING_LOW = 1U,
      DRVRES_FALLING_MEDIUM = 2U,
      DRVRES_FALLING_HIGH = 3U,
      DRVRES_RISING_LOW = 4U,
      DRVRES_RISING_MEDIUM = 5U,
      DRVRES_RISING_HIGH = 6U,
      DRVRES_WHEEL_STOPPER = 7U,
      MAX_NUM_DRVRES_TYPE = 8U,
  };
} // namespace ap_trjctl
#endif // AP_TRJCTL_DRV_RES_TYPE_H_
