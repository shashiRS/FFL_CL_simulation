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

#ifndef ECO_TRACE_DATA_H_
#define ECO_TRACE_DATA_H_

#include "eco/signal_header.h"
#include "Platform_Types.h"


namespace eco
{

  struct TraceData
  {
    ///Signal header indicating data validity
    SignalHeader sigHeader{};
    ///Stores range check failures for trace ports
    uint16 rangeCheckFailures[256]{};
  };

} // namespace eco

#endif // ECO_TRACE_DATA_H_
