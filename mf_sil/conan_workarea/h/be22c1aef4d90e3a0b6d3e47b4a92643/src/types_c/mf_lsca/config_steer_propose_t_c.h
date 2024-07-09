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

#ifndef MF_LSCA_CONFIG_STEER_PROPOSE_T_C_H_
#define MF_LSCA_CONFIG_STEER_PROPOSE_T_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Struct that contains all relevant general parameter data for the objects inside the driving tube
typedef struct
{
    ///The minimum difference between the required left and right steering to avoid an obstacle to make the decision in which direction to steer
    float32 minDiffToAct_deg;
    ///Maximum required steering effort for a steering proposal interference
    float32 maxEffort_deg;
    ///Required driven distance before a steering proposal request is processed
    float32 debounceDist_m;
} MF_LSCA_configSteerPropose_t;

inline MF_LSCA_configSteerPropose_t create_MF_LSCA_configSteerPropose_t(void)
{
  MF_LSCA_configSteerPropose_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_LSCA_CONFIG_STEER_PROPOSE_T_C_H_
