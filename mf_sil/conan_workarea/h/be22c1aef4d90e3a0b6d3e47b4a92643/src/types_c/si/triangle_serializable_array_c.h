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

#ifndef SI_TRIANGLE_SERIALIZABLE_ARRAY_C_H_
#define SI_TRIANGLE_SERIALIZABLE_ARRAY_C_H_

#include "lsm_geoml/size_type_c.h"
#include "si/triangle_serializable_c.h"
#include "eco/memset_c.h"

/// @brief Array of triangles.
typedef struct
{
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU}
    ///@unit{nu}
    ///@brief Describes how many triangles were already added to the array.
    LSM_GEOML_size_type actualSize;
    ///@brief Array containing the triangles.
    SI_TriangleSerializable array[6];
} SI_TriangleSerializableArray;

inline SI_TriangleSerializableArray create_SI_TriangleSerializableArray(void)
{
  SI_TriangleSerializableArray m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.actualSize = 0U;
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.array) / sizeof(m.array[0])); ++i)
    {
      m.array[i] = create_SI_TriangleSerializable();
    }
  }
  return m;
}

#endif // SI_TRIANGLE_SERIALIZABLE_ARRAY_C_H_
