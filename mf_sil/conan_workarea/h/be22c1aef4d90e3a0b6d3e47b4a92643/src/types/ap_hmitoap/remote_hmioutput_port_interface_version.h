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

#ifndef AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_INTERFACE_VERSION_H_
#define AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_hmitoap
{

  struct RemoteHMIOutputPort_InterfaceVersion
  {
    enum { RemoteHMIOutputPort_VERSION = 1U};
  };

  inline ::ap_hmitoap::RemoteHMIOutputPort_InterfaceVersion createRemoteHMIOutputPort_InterfaceVersion()
  {
    RemoteHMIOutputPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(RemoteHMIOutputPort_InterfaceVersion));
    return m;
  }

} // namespace ap_hmitoap

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_hmitoap::RemoteHMIOutputPort_InterfaceVersion create_default()
  {
      return ::ap_hmitoap::createRemoteHMIOutputPort_InterfaceVersion();
  }
}


#endif // AP_HMITOAP_REMOTE_HMIOUTPUT_PORT_INTERFACE_VERSION_H_
