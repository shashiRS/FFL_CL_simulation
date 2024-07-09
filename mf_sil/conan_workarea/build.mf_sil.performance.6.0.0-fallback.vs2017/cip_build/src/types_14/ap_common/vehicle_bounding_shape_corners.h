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

#ifndef AP_COMMON_VEHICLE_BOUNDING_SHAPE_CORNERS_H_
#define AP_COMMON_VEHICLE_BOUNDING_SHAPE_CORNERS_H_

#include "Platform_Types.h"

namespace ap_common
{
  enum class VehicleBoundingShapeCorners : uint8
  {
      FRONT_LEFT_INNER = 0U,
      FRONT_LEFT_OUTER = 1U,
      AXIS_LEFT = 2U,
      REAR_LEFT_OUTER = 3U,
      REAR_LEFT_INNER = 4U,
      REAR_RIGHT_INNER = 5U,
      REAR_RIGHT_OUTER = 6U,
      AXIS_RIGHT = 7U,
      FRONT_RIGHT_OUTER = 8U,
      FRONT_RIGHT_INNER = 9U,
      NUM_RELEVANT_CORNERS = 10U,
  };
} // namespace ap_common
#endif // AP_COMMON_VEHICLE_BOUNDING_SHAPE_CORNERS_H_