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

#ifndef SI_VIRTUAL_LINE_SERIALIZABLE_C_H_
#define SI_VIRTUAL_LINE_SERIALIZABLE_C_H_

#include "si/virt_line_vertices_m_serializable_c.h"
#include "eco/memset_c.h"

/// virtual line constructed to facilitate a smooth orientation alignment of the parked vehicle
typedef struct
{
    ///@unit{m}
    ///start and end point of the virtual line
    SI_VirtLineVertices_mSerializable virtLineVertices_m;
} SI_VirtualLineSerializable;

inline SI_VirtualLineSerializable create_SI_VirtualLineSerializable(void)
{
  SI_VirtualLineSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.virtLineVertices_m = create_SI_VirtLineVertices_mSerializable();
  return m;
}

#endif // SI_VIRTUAL_LINE_SERIALIZABLE_C_H_