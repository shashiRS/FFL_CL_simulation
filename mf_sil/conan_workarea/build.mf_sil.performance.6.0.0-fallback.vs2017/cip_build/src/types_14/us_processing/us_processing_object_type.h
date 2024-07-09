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

#ifndef US_PROCESSING_US_PROCESSING_OBJECT_TYPE_H_
#define US_PROCESSING_US_PROCESSING_OBJECT_TYPE_H_

#include "Platform_Types.h"

namespace us_processing
{
  enum class UsProcessingObjectType : uint8
  {
      US_PROCESSING_OBJ_TYPE_UNKOWN = 0U,
      US_PROCESSING_OBJ_TYPE_POLE = 1U,
      US_PROCESSING_OBJ_TYPE_WALL = 2U,
      US_PROCESSING_OBJ_TYPE_CURB = 3U,
      US_PROCESSING_OBJ_TYPE_VEHICLE = 4U,
      US_PROCESSING_OBJ_TYPE_PEDESTRIAN = 5U,
  };
} // namespace us_processing
#endif // US_PROCESSING_US_PROCESSING_OBJECT_TYPE_H_
