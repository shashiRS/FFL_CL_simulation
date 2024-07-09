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

#ifndef US_PROCESSING_US_PROCESSING_ERRORS_AND_WARNINGS_C_H_
#define US_PROCESSING_US_PROCESSING_ERRORS_AND_WARNINGS_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    uint64 lastErrorsTs;
    uint64 lastWarnsTs;
    uint32 uspErrors;
    uint32 stCfgWarnings;
    uint32 dynCfgWarnings;
    uint32 inputWarnings;
    uint32 sensorWarnings;
    uint32 outputWarnings;
} US_PROCESSING_UsProcessingErrorsAndWarnings;

inline US_PROCESSING_UsProcessingErrorsAndWarnings create_US_PROCESSING_UsProcessingErrorsAndWarnings(void)
{
  US_PROCESSING_UsProcessingErrorsAndWarnings m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // US_PROCESSING_US_PROCESSING_ERRORS_AND_WARNINGS_C_H_