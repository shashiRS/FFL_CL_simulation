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

#ifndef AP_PSM_APP_PSMDEBUG_PORT_INTERFACE_VERSION_H_
#define AP_PSM_APP_PSMDEBUG_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_psm_app
{

  struct PSMDebugPort_InterfaceVersion
  {
    enum { PSMDebugPort_VERSION = 1U};
  };

  inline ::ap_psm_app::PSMDebugPort_InterfaceVersion createPSMDebugPort_InterfaceVersion()
  {
    PSMDebugPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(PSMDebugPort_InterfaceVersion));
    return m;
  }

} // namespace ap_psm_app

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_psm_app::PSMDebugPort_InterfaceVersion create_default()
  {
      return ::ap_psm_app::createPSMDebugPort_InterfaceVersion();
  }
}


#endif // AP_PSM_APP_PSMDEBUG_PORT_INTERFACE_VERSION_H_
