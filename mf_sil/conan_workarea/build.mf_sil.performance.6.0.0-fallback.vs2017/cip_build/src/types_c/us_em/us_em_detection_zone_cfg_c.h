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

#ifndef US_EM_US_EM_DETECTION_ZONE_CFG_C_H_
#define US_EM_US_EM_DETECTION_ZONE_CFG_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    float32 detZoneFrontLeftX;
    float32 detZoneFrontLeftY;
    float32 detZoneFrontRightX;
    float32 detZoneFrontRightY;
    float32 detZoneBackLeftX;
    float32 detZoneBackLeftY;
    float32 detZoneBackRightX;
    float32 detZoneBackRightY;
} US_EM_UsEmDetectionZoneCfg;

inline US_EM_UsEmDetectionZoneCfg create_US_EM_UsEmDetectionZoneCfg(void)
{
  US_EM_UsEmDetectionZoneCfg m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // US_EM_US_EM_DETECTION_ZONE_CFG_C_H_
