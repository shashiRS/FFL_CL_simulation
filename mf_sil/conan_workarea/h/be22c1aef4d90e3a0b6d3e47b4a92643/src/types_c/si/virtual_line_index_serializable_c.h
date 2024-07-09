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

#ifndef SI_VIRTUAL_LINE_INDEX_SERIALIZABLE_C_H_
#define SI_VIRTUAL_LINE_INDEX_SERIALIZABLE_C_H_

#include "lsm_geoml/size_type_c.h"
#include "eco/memset_c.h"

/// @brief Array of virtual line indices.
typedef struct
{
    ///@range{0,16}
    ///@unit{nu}
    ///@brief Describes how many virtual line indices were already added to the array.
    LSM_GEOML_size_type actualSize;
    ///@brief Array containing the virtual line indices.
    LSM_GEOML_size_type array[16];
} SI_VirtualLineIndexSerializable;

inline SI_VirtualLineIndexSerializable create_SI_VirtualLineIndexSerializable(void)
{
  SI_VirtualLineIndexSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.actualSize = 0U;
  return m;
}

#endif // SI_VIRTUAL_LINE_INDEX_SERIALIZABLE_C_H_