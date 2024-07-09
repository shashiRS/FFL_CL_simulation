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

#ifndef MF_LSCA_LSCA_GENERAL_DEBUG_DATA_SERIALIZABLE_C_H_
#define MF_LSCA_LSCA_GENERAL_DEBUG_DATA_SERIALIZABLE_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Debug data regarding general information
typedef struct
{
    ///Indicates if an override action by gas pedal is detected
    boolean gasPedalOverride;
} MF_LSCA_LscaGeneralDebugDataSerializable;

inline MF_LSCA_LscaGeneralDebugDataSerializable create_MF_LSCA_LscaGeneralDebugDataSerializable(void)
{
  MF_LSCA_LscaGeneralDebugDataSerializable m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_LSCA_LSCA_GENERAL_DEBUG_DATA_SERIALIZABLE_C_H_
