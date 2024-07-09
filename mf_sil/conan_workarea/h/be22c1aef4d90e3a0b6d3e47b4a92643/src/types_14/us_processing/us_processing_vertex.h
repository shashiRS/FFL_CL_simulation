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

#ifndef US_PROCESSING_US_PROCESSING_VERTEX_H_
#define US_PROCESSING_US_PROCESSING_VERTEX_H_

#include "Platform_Types.h"


namespace us_processing
{

  struct UsProcessingVertex
  {
    ///@unit{m}
    ///X position of point, cartesian coordinate system DIN 8855 middle of rear axix XY plane parallel to road surface
    float32 posX_m{};
    ///@unit{m}
    ///Y position of point, cartesian coordinate system DIN 8855 middle of rear axix XY plane parallel to road surface
    float32 posY_m{};
    ///@unit{m}
    ///Variance of X position
    float32 posXVar_m{};
    ///@unit{m}
    ///Variance of Y position
    float32 posYVar_m{};
  };

} // namespace us_processing

#endif // US_PROCESSING_US_PROCESSING_VERTEX_H_