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

#ifndef MF_MANAGER_LO_CTRL_REQUEST_TYPE_C_H_
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_C_H_

#include "Platform_Types.h"

typedef uint8 MF_MANAGER_LoCtrlRequestType;

#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_OFF 0U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_BY_TRAJECTORY 1U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_BY_DISTANCE 2U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_BY_VELOCITY 3U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_BY_DISTANCE_VELOCITY 4U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_BY_ACCELERATION 5U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_VELOCITY_LIMITATION 6U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_LOCTRL_EMERGENCY_STOP 7U
#define MF_MANAGER_LO_CTRL_REQUEST_TYPE_MAX_NUM_LO_REQUEST_TYPES 8U


#endif // MF_MANAGER_LO_CTRL_REQUEST_TYPE_C_H_