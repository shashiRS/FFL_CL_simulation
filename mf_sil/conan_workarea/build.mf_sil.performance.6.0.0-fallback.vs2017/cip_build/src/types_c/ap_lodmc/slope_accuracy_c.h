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

#ifndef AP_LODMC_SLOPE_ACCURACY_C_H_
#define AP_LODMC_SLOPE_ACCURACY_C_H_

#include "Platform_Types.h"

///Accurcay of current estimation of dynamic slope.
typedef uint8 AP_LODMC_SlopeAccuracy;

#define AP_LODMC_SLOPE_ACCURACY_HIGH 0U
#define AP_LODMC_SLOPE_ACCURACY_MID 1U
#define AP_LODMC_SLOPE_ACCURACY_LOW 2U
#define AP_LODMC_SLOPE_ACCURACY_UNDEFINED 3U


#endif // AP_LODMC_SLOPE_ACCURACY_C_H_
