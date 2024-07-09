// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef US_PROCESSING_US_PROCESSING_ERRORS_AND_WARNINGS_H_
#define US_PROCESSING_US_PROCESSING_ERRORS_AND_WARNINGS_H_

#include "Platform_Types.h"


namespace us_processing
{

  struct UsProcessingErrorsAndWarnings
  {
    uint64 lastErrorsTs{};
    uint64 lastWarnsTs{};
    uint32 uspErrors{};
    uint32 stCfgWarnings{};
    uint32 dynCfgWarnings{};
    uint32 inputWarnings{};
    uint32 sensorWarnings{};
    uint32 outputWarnings{};
  };

} // namespace us_processing

#endif // US_PROCESSING_US_PROCESSING_ERRORS_AND_WARNINGS_H_
