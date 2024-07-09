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

#ifndef MF_MEMPARK_RECORDING_REQUEST_PORT_C_H_
#define MF_MEMPARK_RECORDING_REQUEST_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Indicates the mapID and the storeRequest
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    uint8 mapID;
    boolean storeRequest;
    uint64 timestamp_us;
} MF_MEMPARK_RecordingRequestPort;

inline MF_MEMPARK_RecordingRequestPort create_MF_MEMPARK_RecordingRequestPort(void)
{
  MF_MEMPARK_RecordingRequestPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.mapID = 255U;
  return m;
}

#endif // MF_MEMPARK_RECORDING_REQUEST_PORT_C_H_
