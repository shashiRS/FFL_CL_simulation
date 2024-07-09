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

#ifndef AP_COMMONVEHSIGPROVIDER_GEARBOX_CTRL_STATUS_PORT_H_
#define AP_COMMONVEHSIGPROVIDER_GEARBOX_CTRL_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_commonvehsigprovider/gear_information.h"
#include "ap_commonvehsigprovider/gear_lever_information.h"
#include "eco/memset.h"


namespace ap_commonvehsigprovider
{

  /// From Gearbox Control.
  struct GearboxCtrlStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@unit{nu}
    ///
    GearInformation gearInformation;
    ///@unit{nu}
    ///
    GearLeverInformation gearLeverInformation;
  };

  inline ::ap_commonvehsigprovider::GearboxCtrlStatusPort createGearboxCtrlStatusPort()
  {
    GearboxCtrlStatusPort m;
    (void)::eco::memset(&m, 0U, sizeof(GearboxCtrlStatusPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.gearInformation = createGearInformation();
    m.gearLeverInformation = createGearLeverInformation();
    return m;
  }

} // namespace ap_commonvehsigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_commonvehsigprovider::GearboxCtrlStatusPort create_default()
  {
      return ::ap_commonvehsigprovider::createGearboxCtrlStatusPort();
  }
}


#endif // AP_COMMONVEHSIGPROVIDER_GEARBOX_CTRL_STATUS_PORT_H_