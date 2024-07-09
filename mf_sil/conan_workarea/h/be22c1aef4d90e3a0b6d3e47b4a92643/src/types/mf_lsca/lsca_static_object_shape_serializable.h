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

#ifndef MF_LSCA_LSCA_STATIC_OBJECT_SHAPE_SERIALIZABLE_H_
#define MF_LSCA_LSCA_STATIC_OBJECT_SHAPE_SERIALIZABLE_H_

#include "Platform_Types.h"
#include "cml/vec2_df_pod.h"
#include "eco/memset.h"


namespace mf_lsca
{

  /// Polygon that contains all coordinates for a static object shape description
  struct LscaStaticObjectShapeSerializable
  {
    ///Number of points in this object shape
    uint32 actualSize;
    ///Points in this shape
    ::cml::Vec2Df_POD points[16];
  };

  inline ::mf_lsca::LscaStaticObjectShapeSerializable createLscaStaticObjectShapeSerializable()
  {
    LscaStaticObjectShapeSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(LscaStaticObjectShapeSerializable));
    {
      const uint64 arraysize = (sizeof(m.points) / sizeof(m.points[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.points[i] = ::cml::createVec2Df_POD();
      }
    }
    return m;
  }

} // namespace mf_lsca

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lsca::LscaStaticObjectShapeSerializable create_default()
  {
      return ::mf_lsca::createLscaStaticObjectShapeSerializable();
  }
}


#endif // MF_LSCA_LSCA_STATIC_OBJECT_SHAPE_SERIALIZABLE_H_
