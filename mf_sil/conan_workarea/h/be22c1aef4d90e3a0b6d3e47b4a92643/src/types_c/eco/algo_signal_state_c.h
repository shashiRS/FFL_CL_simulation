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

#ifndef ECO_ALGO_SIGNAL_STATE_C_H_
#define ECO_ALGO_SIGNAL_STATE_C_H_

#include "Platform_Types.h"

///Algo signal state enumeration  values: enum { AL_SIG_STATE_INIT=0
///,AL_SIG_STATE_OK=1,AL_SIG_STATE_INVALID=2,}
///@range{0,2}
typedef uint8 ECO_AlgoSignalState;

#define ECO_ALGO_SIGNAL_STATE_AL_SIG_STATE_INIT 0U
#define ECO_ALGO_SIGNAL_STATE_AL_SIG_STATE_OK 1U
#define ECO_ALGO_SIGNAL_STATE_AL_SIG_STATE_INVALID 2U

#define ECO_ALGO_SIGNAL_STATE_DEFAULT ECO_ALGO_SIGNAL_STATE_AL_SIG_STATE_INIT

#endif // ECO_ALGO_SIGNAL_STATE_C_H_