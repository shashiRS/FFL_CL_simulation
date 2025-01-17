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

#ifndef AP_PSM_APP_OVERRIDE_LSCAPORT_C_H_
#define AP_PSM_APP_OVERRIDE_LSCAPORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_common/driving_direction_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,1}
    ///Request for distance to STOP from AP and LSCA in order to ignore the object inside the given distance
    float32 ignoreDistanceLSCA_m;
    ///Request for direction of distance to ignore the object in this direction
    AP_COMMON_DrivingDirection ignoreDirection_nu;
} AP_PSM_APP_OverrideLSCAPort;

inline AP_PSM_APP_OverrideLSCAPort create_AP_PSM_APP_OverrideLSCAPort(void)
{
  AP_PSM_APP_OverrideLSCAPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_PSM_APP_OVERRIDE_LSCAPORT_C_H_
