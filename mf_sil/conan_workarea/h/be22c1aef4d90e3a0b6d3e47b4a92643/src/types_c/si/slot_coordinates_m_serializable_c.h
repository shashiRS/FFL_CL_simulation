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

#ifndef SI_SLOT_COORDINATES_M_SERIALIZABLE_C_H_
#define SI_SLOT_COORDINATES_M_SERIALIZABLE_C_H_

#include "lsm_geoml/size_type_c.h"
#include "cml/vec2_df_pod_c.h"
#include "eco/memset_c.h"

typedef struct
{
    LSM_GEOML_size_type actualSize;
    CML_Vec2Df_POD array[4];
} SI_SlotCoordinates_mSerializable;

inline SI_SlotCoordinates_mSerializable create_SI_SlotCoordinates_mSerializable(void)
{
  SI_SlotCoordinates_mSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.array) / sizeof(m.array[0])); ++i)
    {
      m.array[i] = create_CML_Vec2Df_POD();
    }
  }
  return m;
}

#endif // SI_SLOT_COORDINATES_M_SERIALIZABLE_C_H_