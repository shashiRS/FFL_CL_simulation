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

#ifndef US_PROCESSING_US_PROCESSING_DEBUG_PORT_H_
#define US_PROCESSING_US_PROCESSING_DEBUG_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "us_processing/us_processing_errors_and_warnings.h"


namespace us_processing
{

  struct UsProcessingDebugPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///field with information regarding last erros and warnings occured in USP operations
    UsProcessingErrorsAndWarnings uspErrAndWarnings{};
  };

} // namespace us_processing

#endif // US_PROCESSING_US_PROCESSING_DEBUG_PORT_H_
