//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

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

#ifndef COM_COM_VERSION_C_H_
#define COM_COM_VERSION_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// ! \brief Version information struct
///   usage for global version information
typedef struct
{
    uint8 major;
    uint8 minor;
    uint16 patch;
} COM_ComVersion;

inline COM_ComVersion create_COM_ComVersion(void)
{
  COM_ComVersion m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // COM_COM_VERSION_C_H_
