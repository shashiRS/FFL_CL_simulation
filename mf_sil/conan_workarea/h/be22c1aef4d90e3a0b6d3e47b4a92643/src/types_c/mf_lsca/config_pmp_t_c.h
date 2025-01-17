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

#ifndef MF_LSCA_CONFIG_PMP_T_C_H_
#define MF_LSCA_CONFIG_PMP_T_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ///Modeled jerk for pedal kickdown. Assumption: dddot(x) = const = this value while kickdown
    float32 jerkModel_mps;
    ///Safety margin in meters: will be added to the required distance to stop
    float32 safetyMargin_m;
    ///Safety margin in seconds: will be converted into distance by multiplication with current speed
    float32 safetyMargin_s;
    ///If the speed is higher than this value while driving backwards, the function will not intervene
    float32 maxSpeedBackwards_mps;
    ///If the speed is higher than this value while driving forwards, the function will not intervene
    float32 maxSpeedForwards_mps;
} MF_LSCA_configPmp_t;

inline MF_LSCA_configPmp_t create_MF_LSCA_configPmp_t(void)
{
  MF_LSCA_configPmp_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_LSCA_CONFIG_PMP_T_C_H_
