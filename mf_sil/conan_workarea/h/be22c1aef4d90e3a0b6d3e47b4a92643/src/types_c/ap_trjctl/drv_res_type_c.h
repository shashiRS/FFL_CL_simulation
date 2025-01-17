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

#ifndef AP_TRJCTL_DRV_RES_TYPE_C_H_
#define AP_TRJCTL_DRV_RES_TYPE_C_H_

#include "Platform_Types.h"

///Type of the driving resistance. (e.g. for crossing curbstone or approaching wheel stopper)
typedef uint8 AP_TRJCTL_DrvResType;

#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_NONE 0U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_FALLING_LOW 1U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_FALLING_MEDIUM 2U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_FALLING_HIGH 3U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_RISING_LOW 4U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_RISING_MEDIUM 5U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_RISING_HIGH 6U
#define AP_TRJCTL_DRV_RES_TYPE_DRVRES_WHEEL_STOPPER 7U
#define AP_TRJCTL_DRV_RES_TYPE_MAX_NUM_DRVRES_TYPE 8U


#endif // AP_TRJCTL_DRV_RES_TYPE_C_H_
