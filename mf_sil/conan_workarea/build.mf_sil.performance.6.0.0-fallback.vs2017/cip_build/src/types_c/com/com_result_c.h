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

#ifndef COM_COM_RESULT_C_H_
#define COM_COM_RESULT_C_H_

#include "Platform_Types.h"

///! /brief enum for function call results
typedef uint8 COM_ComResult;

///!< Call result ok
#define COM_COM_RESULT_COMRES_OK 0U
///!< wrong configuration, or errors found, run is possible
#define COM_COM_RESULT_COMRES_WARNING 1U
#define COM_COM_RESULT_COMRES_WARN_WRONG_CONFIG 1U
///!< Error no run possible SAVESTATE
#define COM_COM_RESULT_COMRES_ERROR 2U


#endif // COM_COM_RESULT_C_H_
