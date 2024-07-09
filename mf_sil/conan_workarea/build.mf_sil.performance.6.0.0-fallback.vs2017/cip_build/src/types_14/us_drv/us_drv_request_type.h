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

#ifndef US_DRV_US_DRV_REQUEST_TYPE_H_
#define US_DRV_US_DRV_REQUEST_TYPE_H_

#include "Platform_Types.h"

namespace us_drv
{
  enum class UsDrvRequestType : uint8
  {
      ///No active sensor request
      US_REQUEST_TYPE_NONE = 0U,
      ///Active request of type : Sensor Reset
      US_REQUEST_TYPE_RESET = 1U,
      ///Active request of type : Sensor Sleep Mode
      US_REQUEST_TYPE_SLEEP = 2U,
      ///Active request of type : Sensor Safe State
      US_REQUEST_TYPE_SAFESTATE = 3U,
      ///Active request of type : Update Measurement Mode
      US_REQUEST_TYPE_UMM = 4U,
  };
} // namespace us_drv
#endif // US_DRV_US_DRV_REQUEST_TYPE_H_
