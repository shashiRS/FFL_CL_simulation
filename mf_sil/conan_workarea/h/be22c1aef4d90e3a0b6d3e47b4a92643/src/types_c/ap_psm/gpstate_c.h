//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

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

#ifndef AP_PSM_GPSTATE_C_H_
#define AP_PSM_GPSTATE_C_H_

#include "Platform_Types.h"

///State of the Garage Parking.
typedef uint8 AP_PSM_GPState;

#define AP_PSM_GPSTATE_GP_INACTIVE 0U
#define AP_PSM_GPSTATE_GP_SCAN_IN 1U
#define AP_PSM_GPSTATE_GP_SCAN_OUT 2U
#define AP_PSM_GPSTATE_GP_AVG_ACTIVE_IN 3U
#define AP_PSM_GPSTATE_GP_AVG_ACTIVE_OUT 4U
#define AP_PSM_GPSTATE_GP_AVG_PAUSE 5U
#define AP_PSM_GPSTATE_GP_AVG_UNDO 6U
#define AP_PSM_GPSTATE_GP_AVG_FINISHED 7U


#endif // AP_PSM_GPSTATE_C_H_
