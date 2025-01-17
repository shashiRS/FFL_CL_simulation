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

#ifndef MF_HMIH_WHEEL_WARNINGS_C_H_
#define MF_HMIH_WHEEL_WARNINGS_C_H_

#include "mf_whlprotectproc/whl_warning_level_c.h"
#include "Platform_Types.h"
#include "mf_hmih/wheel_ang_direction_c.h"
#include "eco/memset_c.h"

/// Wheel warning information coming from WHP
typedef struct
{
    ///@range{0,3}
    ///Wheel warning level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
    MF_WHLPROTECTPROC_WhlWarningLevel warningLevel_nu[4];
    ///@unit{deg}
    ///Absolute value of wheel angles for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
    uint8 whlAngleAbs_deg[4];
    ///@range{0,1}
    ///Direction of the wheel angle for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3.
    MF_HMIH_WheelAngDirection whlAngDirection_nu[4];
} MF_HMIH_WheelWarnings;

inline MF_HMIH_WheelWarnings create_MF_HMIH_WheelWarnings(void)
{
  MF_HMIH_WheelWarnings m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_HMIH_WHEEL_WARNINGS_C_H_
