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

#ifndef SI_OBJECT_SHAPE_SERIALIZABLE_H_
#define SI_OBJECT_SHAPE_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "cml/vec2_df_pod.h"


namespace si
{

  /// @brief Array of vertices representing an object.
  struct ObjectShapeSerializable
  {
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PTS_STATIC_POLY_NU}
    ///@unit{nu}
    ///@brief Describes how many vertices were already added to the array.
    ::lsm_geoml::size_type actualSize{0U};
    ///@brief Array containing the vertices.
    ::cml::Vec2Df_POD array[10]{};
  };

} // namespace si

#endif // SI_OBJECT_SHAPE_SERIALIZABLE_H_
