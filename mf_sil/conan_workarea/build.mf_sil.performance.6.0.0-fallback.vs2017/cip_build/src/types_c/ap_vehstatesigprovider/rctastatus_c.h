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

#ifndef AP_VEHSTATESIGPROVIDER_RCTASTATUS_C_H_
#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_C_H_

#include "Platform_Types.h"

///Rear cross traffic alert status
typedef uint8 AP_VEHSTATESIGPROVIDER_RCTAStatus;

#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_RCTA_OFF 0U
#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_RCTA_PASSIVE 1U
#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_RCTA_ACTIVE_PREWARNING 2U
#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_RCTA_ACTIVE_ACUTE_WARNING 3U
#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_RCTA_ACTIVE_BRAKING 4U
#define AP_VEHSTATESIGPROVIDER_RCTASTATUS_RCTA_ERROR 5U


#endif // AP_VEHSTATESIGPROVIDER_RCTASTATUS_C_H_
