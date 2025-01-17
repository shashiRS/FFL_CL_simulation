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

#ifndef COM_EBOOL_T_C_H_
#define COM_EBOOL_T_C_H_

#include "Platform_Types.h"

///   ! \brief ebool typedef for export or measurement struct
///full ranges from 0..0xFF is defined
///!! never use bool   only  0 and 1 is defined
typedef uint8 COM_ebool_t;

///!< FALSE_e
#define COM_EBOOL_T_FALSE_e 0U
///!< TRUE_e
#define COM_EBOOL_T_TRUE_e 1U


#endif // COM_EBOOL_T_C_H_
