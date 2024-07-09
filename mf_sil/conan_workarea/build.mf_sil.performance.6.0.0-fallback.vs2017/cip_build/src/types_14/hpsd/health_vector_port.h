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

#ifndef HPSD_HEALTH_VECTOR_PORT_H_
#define HPSD_HEALTH_VECTOR_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "hpsd/health_vector.h"


namespace hpsd
{

  /// A port structure containing the health vector.
  struct HealthVectorPort
  {
    ///Version number of interface.
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ///Signal header with common signal information.
    ::eco::SignalHeader sSigHeader{};
    ///A list of booleans describing the consolidated health status of the system.
    HealthVector healthVector{};
  };

} // namespace hpsd

#endif // HPSD_HEALTH_VECTOR_PORT_H_