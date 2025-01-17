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

#ifndef MF_LSCA_BRAKE_MODEL_PARAMS_T_C_H_
#define MF_LSCA_BRAKE_MODEL_PARAMS_T_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Struct that contains all relevant brake model parameter data for one specific brake exeuction scenario
typedef struct
{
    ///Deadtime from lsca braking flag to first increase of brake pressure
    float32 deadtime_s;
    ///Ramp time for linear increase until maximum brake pressure
    float32 rampTime_s;
    ///Maximum brake deceleration
    float32 maxDeceleration_ms;
} MF_LSCA_brakeModelParams_t;

inline MF_LSCA_brakeModelParams_t create_MF_LSCA_brakeModelParams_t(void)
{
  MF_LSCA_brakeModelParams_t m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_LSCA_BRAKE_MODEL_PARAMS_T_C_H_
