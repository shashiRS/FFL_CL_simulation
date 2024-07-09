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

#ifndef MF_MANAGER_ACTIVE_MANEUVERING_FUNCTION_PORT_H_
#define MF_MANAGER_ACTIVE_MANEUVERING_FUNCTION_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_manager/active_maneuvering_function.h"
#include "eco/memset.h"


namespace mf_manager
{

  struct ActiveManeuveringFunctionPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ActiveManeuveringFunction loCtrlRequestOrigin;
    ActiveManeuveringFunction laCtrlRequestOrigin;
  };

  inline ::mf_manager::ActiveManeuveringFunctionPort createActiveManeuveringFunctionPort()
  {
    ActiveManeuveringFunctionPort m;
    (void)::eco::memset(&m, 0U, sizeof(ActiveManeuveringFunctionPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace mf_manager

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_manager::ActiveManeuveringFunctionPort create_default()
  {
      return ::mf_manager::createActiveManeuveringFunctionPort();
  }
}


#endif // MF_MANAGER_ACTIVE_MANEUVERING_FUNCTION_PORT_H_