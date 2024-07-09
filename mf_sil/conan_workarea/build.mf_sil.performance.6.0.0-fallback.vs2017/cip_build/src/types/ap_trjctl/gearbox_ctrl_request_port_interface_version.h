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

#ifndef AP_TRJCTL_GEARBOX_CTRL_REQUEST_PORT_INTERFACE_VERSION_H_
#define AP_TRJCTL_GEARBOX_CTRL_REQUEST_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_trjctl
{

  struct GearboxCtrlRequestPort_InterfaceVersion
  {
    enum { GearboxCtrlRequestPort_VERSION = 1U};
  };

  inline ::ap_trjctl::GearboxCtrlRequestPort_InterfaceVersion createGearboxCtrlRequestPort_InterfaceVersion()
  {
    GearboxCtrlRequestPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(GearboxCtrlRequestPort_InterfaceVersion));
    return m;
  }

} // namespace ap_trjctl

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_trjctl::GearboxCtrlRequestPort_InterfaceVersion create_default()
  {
      return ::ap_trjctl::createGearboxCtrlRequestPort_InterfaceVersion();
  }
}


#endif // AP_TRJCTL_GEARBOX_CTRL_REQUEST_PORT_INTERFACE_VERSION_H_