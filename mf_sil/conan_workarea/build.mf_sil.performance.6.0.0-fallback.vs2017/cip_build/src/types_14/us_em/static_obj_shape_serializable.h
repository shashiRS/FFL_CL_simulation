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

#ifndef US_EM_STATIC_OBJ_SHAPE_SERIALIZABLE_H_
#define US_EM_STATIC_OBJ_SHAPE_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "cml/vec2_df_pod.h"


namespace us_em
{

  struct StaticObjShapeSerializable
  {
    ///@unit{nu}
    ///@range{0,US_EM_MAX_NUM_STATIC_OBJ_PTS}
    ///Number of vertices in object polygon
    ::lsm_geoml::size_type actualSize{};
    ///@unit{Vector2D}
    ///@range{-1000.0F,1000.0F}
    ///Vertex x,y coordinates
    ::cml::Vec2Df_POD array[10]{};
  };

} // namespace us_em

#endif // US_EM_STATIC_OBJ_SHAPE_SERIALIZABLE_H_