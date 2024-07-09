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

#ifndef MF_MEMPARK_MEMORY_PARKING_STATUS_PORT_C_H_
#define MF_MEMPARK_MEMORY_PARKING_STATUS_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "mf_mempark/mpstatus_c.h"
#include "mf_mempark/tpstatus_c.h"
#include "mf_mempark/storing_status_c.h"
#include "eco/memset_c.h"

/// The elements for MemoryParkingStatusPort
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    MF_MEMPARK_MPStatus mpStatus;
    MF_MEMPARK_TPStatus tpStatus;
    MF_MEMPARK_StoringStatus storingStatus;
} MF_MEMPARK_MemoryParkingStatusPort;

inline MF_MEMPARK_MemoryParkingStatusPort create_MF_MEMPARK_MemoryParkingStatusPort(void)
{
  MF_MEMPARK_MemoryParkingStatusPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.mpStatus = create_MF_MEMPARK_MPStatus();
  m.tpStatus = create_MF_MEMPARK_TPStatus();
  return m;
}

#endif // MF_MEMPARK_MEMORY_PARKING_STATUS_PORT_C_H_
