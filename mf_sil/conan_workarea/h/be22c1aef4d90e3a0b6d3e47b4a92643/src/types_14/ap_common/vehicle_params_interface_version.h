// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

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

#ifndef AP_COMMON_VEHICLE_PARAMS_INTERFACE_VERSION_H_
#define AP_COMMON_VEHICLE_PARAMS_INTERFACE_VERSION_H_

#include "Platform_Types.h"


namespace ap_common
{

  struct Vehicle_Params_InterfaceVersion
  {
    static constexpr uint32 Vehicle_Params_VERSION = 1U;
  };

} // namespace ap_common

#endif // AP_COMMON_VEHICLE_PARAMS_INTERFACE_VERSION_H_
