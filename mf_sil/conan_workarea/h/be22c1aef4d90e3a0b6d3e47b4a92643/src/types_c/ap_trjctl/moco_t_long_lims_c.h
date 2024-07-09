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

#ifndef AP_TRJCTL_MOCO_T_LONG_LIMS_C_H_
#define AP_TRJCTL_MOCO_T_LONG_LIMS_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Longitudinal limitations of acceleration and jerk (MoCo variant only)
typedef struct
{
    ///Upper limit for longitudinal acceleration
    float32 longAccelMax;
    ///Lower limit for longitudinal acceleration
    float32 longAccelMin;
    ///Upper limit for longitudinal jerk
    float32 longJerkMax;
    ///Lower limit for longitudinal jerk
    float32 longJerkMin;
} AP_TRJCTL_MOCO_t_LongLims;

inline AP_TRJCTL_MOCO_t_LongLims create_AP_TRJCTL_MOCO_t_LongLims(void)
{
  AP_TRJCTL_MOCO_t_LongLims m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // AP_TRJCTL_MOCO_T_LONG_LIMS_C_H_
