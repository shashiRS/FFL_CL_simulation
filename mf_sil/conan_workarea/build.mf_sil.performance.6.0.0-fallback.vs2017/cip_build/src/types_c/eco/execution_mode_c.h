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

#ifndef ECO_EXECUTION_MODE_C_H_
#define ECO_EXECUTION_MODE_C_H_

#include "eco/signal_header_c.h"
#include "eco/component_execution_mode_c.h"
#include "eco/memset_c.h"

/// Execution information needed to execute a component
typedef struct
{
    ///ADAS signal header
    ECO_SignalHeader sigHeader;
    ///The component execution mode and sub mode information
    ECO_ComponentExecutionMode mode;
} ECO_ExecutionMode;

inline ECO_ExecutionMode create_ECO_ExecutionMode(void)
{
  ECO_ExecutionMode m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sigHeader = create_ECO_SignalHeader();
  m.mode = create_ECO_ComponentExecutionMode();
  return m;
}

#endif // ECO_EXECUTION_MODE_C_H_
