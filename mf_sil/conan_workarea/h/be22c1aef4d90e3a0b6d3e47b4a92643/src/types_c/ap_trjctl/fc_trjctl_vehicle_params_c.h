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

#ifndef AP_TRJCTL_FC_TRJCTL_VEHICLE_PARAMS_C_H_
#define AP_TRJCTL_FC_TRJCTL_VEHICLE_PARAMS_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ///Maximum steering angle (at wheels).
    float32 AP_V_MAX_STEER_ANG_RAD;
} AP_TRJCTL_FC_TRJCTL_Vehicle_Params;

inline AP_TRJCTL_FC_TRJCTL_Vehicle_Params create_AP_TRJCTL_FC_TRJCTL_Vehicle_Params(void)
{
  AP_TRJCTL_FC_TRJCTL_Vehicle_Params m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // AP_TRJCTL_FC_TRJCTL_VEHICLE_PARAMS_C_H_
