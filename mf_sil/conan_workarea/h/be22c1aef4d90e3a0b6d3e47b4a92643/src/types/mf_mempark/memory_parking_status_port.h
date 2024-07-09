// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef MF_MEMPARK_MEMORY_PARKING_STATUS_PORT_H_
#define MF_MEMPARK_MEMORY_PARKING_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_mempark/mpstatus.h"
#include "mf_mempark/tpstatus.h"
#include "mf_mempark/storing_status.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// The elements for MemoryParkingStatusPort
  struct MemoryParkingStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    MPStatus mpStatus;
    TPStatus tpStatus;
    StoringStatus storingStatus;
  };

  inline ::mf_mempark::MemoryParkingStatusPort createMemoryParkingStatusPort()
  {
    MemoryParkingStatusPort m;
    (void)::eco::memset(&m, 0U, sizeof(MemoryParkingStatusPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.mpStatus = createMPStatus();
    m.tpStatus = createTPStatus();
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::MemoryParkingStatusPort create_default()
  {
      return ::mf_mempark::createMemoryParkingStatusPort();
  }
}


#endif // MF_MEMPARK_MEMORY_PARKING_STATUS_PORT_H_
