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

#ifndef ECO_TRACE_DATA_C_H_
#define ECO_TRACE_DATA_C_H_

#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ///Signal header indicating data validity
    ECO_SignalHeader sigHeader;
    ///Stores range check failures for trace ports
    uint16 rangeCheckFailures[256];
} ECO_TraceData;

inline ECO_TraceData create_ECO_TraceData(void)
{
  ECO_TraceData m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // ECO_TRACE_DATA_C_H_
