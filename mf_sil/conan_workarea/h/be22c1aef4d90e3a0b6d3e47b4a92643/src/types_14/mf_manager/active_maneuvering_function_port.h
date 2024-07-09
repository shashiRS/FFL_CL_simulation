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

#ifndef MF_MANAGER_ACTIVE_MANEUVERING_FUNCTION_PORT_H_
#define MF_MANAGER_ACTIVE_MANEUVERING_FUNCTION_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_manager/active_maneuvering_function.h"


namespace mf_manager
{

  struct ActiveManeuveringFunctionPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ActiveManeuveringFunction loCtrlRequestOrigin{};
    ActiveManeuveringFunction laCtrlRequestOrigin{};
  };

} // namespace mf_manager

#endif // MF_MANAGER_ACTIVE_MANEUVERING_FUNCTION_PORT_H_
