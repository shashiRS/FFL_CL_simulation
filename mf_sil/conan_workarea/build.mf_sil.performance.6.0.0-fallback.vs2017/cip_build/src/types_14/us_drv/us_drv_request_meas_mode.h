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

#ifndef US_DRV_US_DRV_REQUEST_MEAS_MODE_H_
#define US_DRV_US_DRV_REQUEST_MEAS_MODE_H_

#include "Platform_Types.h"

namespace us_drv
{
  enum class UsDrvRequestMeasMode : uint8
  {
      ///Measurement mode set to near range echoes
      US_MEASMODE_NEAR_RANGE_FIELD = 0U,
      ///Measurement mode set to far range echoes
      US_MEASMODE_FAR_RANGE_FIELD = 1U,
      ///Number of measurement modes and value used when no mode is available due to sensor not running
      US_MEASMODE_COUNT = 2U,
  };
} // namespace us_drv
#endif // US_DRV_US_DRV_REQUEST_MEAS_MODE_H_