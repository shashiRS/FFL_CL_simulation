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

#ifndef MF_MEMPARK_MEMORY_PARKING_DATA_RESULT_PORT_H_
#define MF_MEMPARK_MEMORY_PARKING_DATA_RESULT_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "mf_mempark/storing_status.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// The response from the Parameter Handler to a request to perform an operation.
  struct MemoryParkingDataResultPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///The unique identifier of the saved map.
    uint8 metaMapIdSaving;
    ///The state of the save request.
    StoringStatus savingStatus;
    ///The unique identifier of the loaded map.
    uint8 metaMapIdLoading;
    ///The state of the load request.
    StoringStatus loadingStatus;
    ///The unique identifier of the deleted map.
    uint8 metaMapIdDeleting;
    ///The state of the delete request.
    StoringStatus deletingStatus;
  };

  inline ::mf_mempark::MemoryParkingDataResultPort createMemoryParkingDataResultPort()
  {
    MemoryParkingDataResultPort m;
    (void)::eco::memset(&m, 0U, sizeof(MemoryParkingDataResultPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::MemoryParkingDataResultPort create_default()
  {
      return ::mf_mempark::createMemoryParkingDataResultPort();
  }
}


#endif // MF_MEMPARK_MEMORY_PARKING_DATA_RESULT_PORT_H_
