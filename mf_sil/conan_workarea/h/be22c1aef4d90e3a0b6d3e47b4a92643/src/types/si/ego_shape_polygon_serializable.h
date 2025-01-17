// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef SI_EGO_SHAPE_POLYGON_SERIALIZABLE_H_
#define SI_EGO_SHAPE_POLYGON_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "cml/vec2_df_pod.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Array of vertices.
  struct EgoShapePolygonSerializable
  {
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_V_VEHICLE_SHAPE_MAX_SIZE_NU}
    ///@unit{nu}
    ///@brief Describes how many vertices were already added to the array.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array containing the vertices.
    ::cml::Vec2Df_POD array[16];
  };

  inline ::si::EgoShapePolygonSerializable createEgoShapePolygonSerializable()
  {
    EgoShapePolygonSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(EgoShapePolygonSerializable));
    m.actualSize = 0U;
    {
      const uint64 arraysize = (sizeof(m.array) / sizeof(m.array[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.array[i] = ::cml::createVec2Df_POD();
      }
    }
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::EgoShapePolygonSerializable create_default()
  {
      return ::si::createEgoShapePolygonSerializable();
  }
}


#endif // SI_EGO_SHAPE_POLYGON_SERIALIZABLE_H_
