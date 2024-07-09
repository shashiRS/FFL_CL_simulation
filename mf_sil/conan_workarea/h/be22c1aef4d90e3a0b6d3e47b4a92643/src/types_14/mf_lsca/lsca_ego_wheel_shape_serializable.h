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

#ifndef MF_LSCA_LSCA_EGO_WHEEL_SHAPE_SERIALIZABLE_H_
#define MF_LSCA_LSCA_EGO_WHEEL_SHAPE_SERIALIZABLE_H_

#include "Platform_Types.h"
#include "cml/vec2_df_pod.h"


namespace mf_lsca
{

  /// Polygon that contains all coordinates for the wheel shape description
  struct LscaEgoWheelShapeSerializable
  {
    ///Number of points in this shape
    uint32 actualSize{};
    ///Points in this shape
    ::cml::Vec2Df_POD points[4]{};
  };

} // namespace mf_lsca

#endif // MF_LSCA_LSCA_EGO_WHEEL_SHAPE_SERIALIZABLE_H_
