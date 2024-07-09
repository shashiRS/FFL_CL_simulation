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

#ifndef MF_MANAGER_LA_CTRL_REQUEST_TYPE_C_H_
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_C_H_

#include "Platform_Types.h"

typedef uint8 MF_MANAGER_LaCtrlRequestType;

#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_OFF 0U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_BY_TRAJECTORY 1U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_BY_ANGLE_FRONT 2U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_BY_ANGLE_REAR 3U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_BY_ANGLE_FRONT_REAR 4U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_BY_TORQUE 5U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_BY_CURVATURE 6U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_LACTRL_COMF_ANGLE_ADJUSTMENT 7U
#define MF_MANAGER_LA_CTRL_REQUEST_TYPE_MAX_NUM_LA_REQUEST_TYPES 8U


#endif // MF_MANAGER_LA_CTRL_REQUEST_TYPE_C_H_
