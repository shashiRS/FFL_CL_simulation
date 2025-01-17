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

#ifndef AP_COMMON_EGO_VEHICLE_SHAPE_TYPE_H_
#define AP_COMMON_EGO_VEHICLE_SHAPE_TYPE_H_

#include "Platform_Types.h"

namespace ap_common
{
  enum class EgoVehicleShapeType : uint8
  {
      EGO_VEH_SHAPE_STANDARD = 0U,
      EGO_VEH_SHAPE_BOUNDING = 1U,
  };
} // namespace ap_common
#endif // AP_COMMON_EGO_VEHICLE_SHAPE_TYPE_H_
