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

#ifndef SI_QUADRILATERAL_SERIALIZABLE_H_
#define SI_QUADRILATERAL_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "cml/vec2_df_pod.h"
#include "eco/memset.h"


namespace si
{

  /// @brief A structure describing a shape with four vertices.
  struct QuadrilateralSerializable
  {
    ///@range{0,4}
    ///@unit{nu}
    ///@brief Describes how many vertices were already added to the quadrilateral.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array containing the vertices.
    ::cml::Vec2Df_POD array[4];
  };

  inline ::si::QuadrilateralSerializable createQuadrilateralSerializable()
  {
    QuadrilateralSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(QuadrilateralSerializable));
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
  inline ::si::QuadrilateralSerializable create_default()
  {
      return ::si::createQuadrilateralSerializable();
  }
}


#endif // SI_QUADRILATERAL_SERIALIZABLE_H_