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

#ifndef HPSD_PLATFORM_STATE_PORT_H_
#define HPSD_PLATFORM_STATE_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "hpsd/platform_state.h"
#include "eco/memset.h"


namespace hpsd
{

  /// Port structure that describes the state of the platform
  struct PlatformStatePort
  {
    ///Version number of interface.
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ///Signal header with common signal information.
    ::eco::SignalHeader sSigHeader;
    ///Enumeration describing the current state of the platform.
    PlatformState platformState;
  };

  inline ::hpsd::PlatformStatePort createPlatformStatePort()
  {
    PlatformStatePort m;
    (void)::eco::memset(&m, 0U, sizeof(PlatformStatePort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace hpsd

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::hpsd::PlatformStatePort create_default()
  {
      return ::hpsd::createPlatformStatePort();
  }
}


#endif // HPSD_PLATFORM_STATE_PORT_H_
