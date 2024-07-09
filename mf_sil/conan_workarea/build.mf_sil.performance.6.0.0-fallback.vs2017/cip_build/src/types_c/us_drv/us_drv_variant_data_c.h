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

#ifndef US_DRV_US_DRV_VARIANT_DATA_C_H_
#define US_DRV_US_DRV_VARIANT_DATA_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    uint8 usedAsicNum;
    uint8 usedSensorNum;
} US_DRV_UsDrvVariantData;

inline US_DRV_UsDrvVariantData create_US_DRV_UsDrvVariantData(void)
{
  US_DRV_UsDrvVariantData m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // US_DRV_US_DRV_VARIANT_DATA_C_H_
