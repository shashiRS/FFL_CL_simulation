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

#ifndef AP_PSM_APP_GARAGE_PARKING_H_
#define AP_PSM_APP_GARAGE_PARKING_H_

#include "Platform_Types.h"

namespace ap_psm_app
{
  enum class GarageParking : uint8
  {
      GP_NOT_AVAILABLE = 0U,
      GP_IN_SCAN_FWD = 1U,
      GP_IN_SCAN_BWD = 2U,
      GP_OUT_FWD = 3U,
      GP_OUT_BWD = 4U,
      GP_IN_DETECTED_FWD = 5U,
      GP_IN_DETECTED_BWD = 6U,
  };
} // namespace ap_psm_app
#endif // AP_PSM_APP_GARAGE_PARKING_H_
