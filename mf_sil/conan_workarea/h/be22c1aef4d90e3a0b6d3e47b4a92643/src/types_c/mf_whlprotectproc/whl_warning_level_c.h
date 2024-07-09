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

#ifndef MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_C_H_
#define MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_C_H_

#include "Platform_Types.h"

///Wheel warning level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3, to be defined in an enum.
typedef uint8 MF_WHLPROTECTPROC_WhlWarningLevel;

#define MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_WHP_NOT_AVAILABLE 0U
#define MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_WHP_NO_WARNING 1U
#define MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_WHP_LOW_WARNING 2U
#define MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_WHP_HIGH_WARNING 3U


#endif // MF_WHLPROTECTPROC_WHL_WARNING_LEVEL_C_H_