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

#ifndef US_PROCESSING_US_PROCESSING_FILTERED_ECHO_LIST_H_
#define US_PROCESSING_US_PROCESSING_FILTERED_ECHO_LIST_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "us_processing/us_processing_filtered_echo.h"


namespace us_processing
{

  struct UsProcessingFilteredEchoList
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@range{0,US_PROCESSING_MAX_NUM_ECHOES}
    ///@unit{nu}
    ///Number of elements in echoes array
    uint16 numEchoes{};
    ///Array of filtered ultrasonic echoes
    UsProcessingFilteredEcho echoes[720]{};
  };

} // namespace us_processing

#endif // US_PROCESSING_US_PROCESSING_FILTERED_ECHO_LIST_H_