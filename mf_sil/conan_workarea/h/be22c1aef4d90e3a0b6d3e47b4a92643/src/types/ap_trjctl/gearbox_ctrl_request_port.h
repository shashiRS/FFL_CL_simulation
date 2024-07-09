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

#ifndef AP_TRJCTL_GEARBOX_CTRL_REQUEST_PORT_H_
#define AP_TRJCTL_GEARBOX_CTRL_REQUEST_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_commonvehsigprovider/gear.h"
#include "eco/memset.h"


namespace ap_trjctl
{

  /// Request to Gearbox Control.
  struct GearboxCtrlRequestPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Control command for the activation/deactivation of the Gearbox Control.
    boolean gearboxCtrlRequest_nu;
    ///Set gear switch request.
    boolean gearSwitchRequest_nu;
    ///@range{0,15}
    ///Requested gear.
    ::ap_commonvehsigprovider::Gear gearReq_nu;
  };

  inline ::ap_trjctl::GearboxCtrlRequestPort createGearboxCtrlRequestPort()
  {
    GearboxCtrlRequestPort m;
    (void)::eco::memset(&m, 0U, sizeof(GearboxCtrlRequestPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.gearReq_nu = ::ap_commonvehsigprovider::Gear::GEAR_NOT_DEFINED;
    return m;
  }

} // namespace ap_trjctl

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_trjctl::GearboxCtrlRequestPort create_default()
  {
      return ::ap_trjctl::createGearboxCtrlRequestPort();
  }
}


#endif // AP_TRJCTL_GEARBOX_CTRL_REQUEST_PORT_H_
