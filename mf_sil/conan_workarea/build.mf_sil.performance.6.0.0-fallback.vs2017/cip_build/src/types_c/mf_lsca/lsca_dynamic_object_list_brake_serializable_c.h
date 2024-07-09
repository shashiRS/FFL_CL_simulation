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

#ifndef MF_LSCA_LSCA_DYNAMIC_OBJECT_LIST_BRAKE_SERIALIZABLE_C_H_
#define MF_LSCA_LSCA_DYNAMIC_OBJECT_LIST_BRAKE_SERIALIZABLE_C_H_

#include "Platform_Types.h"
#include "mf_lsca/lsca_dynamic_object_shape_serializable_c.h"
#include "eco/memset_c.h"

/// Collection of dynamic objects for debug purposes
typedef struct
{
    ///Number of objects in this list
    uint32 actualSize;
    ///Objects in this list
    MF_LSCA_LscaDynamicObjectShapeSerializable objects[10];
} MF_LSCA_LscaDynamicObjectListBrakeSerializable;

inline MF_LSCA_LscaDynamicObjectListBrakeSerializable create_MF_LSCA_LscaDynamicObjectListBrakeSerializable(void)
{
  MF_LSCA_LscaDynamicObjectListBrakeSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.objects) / sizeof(m.objects[0])); ++i)
    {
      m.objects[i] = create_MF_LSCA_LscaDynamicObjectShapeSerializable();
    }
  }
  return m;
}

#endif // MF_LSCA_LSCA_DYNAMIC_OBJECT_LIST_BRAKE_SERIALIZABLE_C_H_
