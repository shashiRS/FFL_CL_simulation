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

#ifndef AP_PSM_APP_VSMSTATE_H_
#define AP_PSM_APP_VSMSTATE_H_

#include "Platform_Types.h"

namespace ap_psm_app
{
  enum class VSMState : uint8
  {
      VSM_INACTIVE = 0U,
      VSM_SCANNING_CONDITIONS_NOT_MET = 1U,
      VSM_SCANNING_CONDITIONS_MET = 2U,
      VSM_MANEUVERING_CONDITIONS_MET = 3U,
  };
} // namespace ap_psm_app
#endif // AP_PSM_APP_VSMSTATE_H_
