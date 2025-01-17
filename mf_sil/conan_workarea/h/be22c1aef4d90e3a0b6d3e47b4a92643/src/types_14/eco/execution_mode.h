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

#ifndef ECO_EXECUTION_MODE_H_
#define ECO_EXECUTION_MODE_H_

#include "eco/signal_header.h"
#include "eco/component_execution_mode.h"


namespace eco
{

  /// Execution information needed to execute a component
  struct ExecutionMode
  {
    ///ADAS signal header
    SignalHeader sigHeader{};
    ///The component execution mode and sub mode information
    ComponentExecutionMode mode{};
  };

} // namespace eco

#endif // ECO_EXECUTION_MODE_H_
