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

#ifndef ECU_CTRL_ECU_STATE_C_H_
#define ECU_CTRL_ECU_STATE_C_H_

#include "Platform_Types.h"

typedef uint8 ECU_CTRL_EcuState;

///ECU_STATE_INIT data or struct is not valid
#define ECU_CTRL_ECU_STATE_ECU_STATE_INIT 0U
///ECU_STATE_STARTUP data is valid for other usage
#define ECU_CTRL_ECU_STATE_ECU_STATE_STARTUP 1U
///ECU_STATE_RUNNING data is invalid or out of range, use with care
#define ECU_CTRL_ECU_STATE_ECU_STATE_RUNNING 2U
///ECU_STATE_SHUTDOWN data in timeout state
#define ECU_CTRL_ECU_STATE_ECU_STATE_SHUTDOWN 3U
///ECU_STATE_UNKNOWN data in error state
#define ECU_CTRL_ECU_STATE_ECU_STATE_UNKNOWN 128U


#endif // ECU_CTRL_ECU_STATE_C_H_