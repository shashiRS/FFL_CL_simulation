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

#ifndef AP_PSM_MOTION_CONTROL_REQUEST_TYPE_C_H_
#define AP_PSM_MOTION_CONTROL_REQUEST_TYPE_C_H_

#include "Platform_Types.h"

///Define the request type for the Motion control.
typedef uint8 AP_PSM_MotionControlRequestType;

#define AP_PSM_MOTION_CONTROL_REQUEST_TYPE_PSM_MCRT_NORMAL_REQUEST 0U
#define AP_PSM_MOTION_CONTROL_REQUEST_TYPE_PSM_MCRT_OVERRIDE_REQUEST 1U
#define AP_PSM_MOTION_CONTROL_REQUEST_TYPE_PSM_MCRT_DEGRADATION_REQUEST 2U


#endif // AP_PSM_MOTION_CONTROL_REQUEST_TYPE_C_H_
