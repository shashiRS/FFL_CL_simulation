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

#ifndef MF_MEMPARK_DELETION_REQUEST_PORT_H_
#define MF_MEMPARK_DELETION_REQUEST_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"


namespace mf_mempark
{

  /// Indicates the mapID and the deleteRequest
  struct DeletionRequestPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///ID of map that will be deleted; if INVALID_MAP_ID, no processing takes place
    uint8 mapID{255U};
  };

} // namespace mf_mempark

#endif // MF_MEMPARK_DELETION_REQUEST_PORT_H_