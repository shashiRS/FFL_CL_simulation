//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef SI_EGO_SHAPE_POLYGON_SERIALIZABLE_C_H_
#define SI_EGO_SHAPE_POLYGON_SERIALIZABLE_C_H_

#include "lsm_geoml/size_type_c.h"
#include "cml/vec2_df_pod_c.h"
#include "eco/memset_c.h"

/// @brief Array of vertices.
typedef struct
{
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_V_VEHICLE_SHAPE_MAX_SIZE_NU}
    ///@unit{nu}
    ///@brief Describes how many vertices were already added to the array.
    LSM_GEOML_size_type actualSize;
    ///@brief Array containing the vertices.
    CML_Vec2Df_POD array[16];
} SI_EgoShapePolygonSerializable;

inline SI_EgoShapePolygonSerializable create_SI_EgoShapePolygonSerializable(void)
{
  SI_EgoShapePolygonSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.actualSize = 0U;
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.array) / sizeof(m.array[0])); ++i)
    {
      m.array[i] = create_CML_Vec2Df_POD();
    }
  }
  return m;
}

#endif // SI_EGO_SHAPE_POLYGON_SERIALIZABLE_C_H_