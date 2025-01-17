//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef AP_COMMONVEHSIGPROVIDER_GEARBOX_CTRL_STATUS_PORT_C_H_
#define AP_COMMONVEHSIGPROVIDER_GEARBOX_CTRL_STATUS_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "ap_commonvehsigprovider/gear_information_c.h"
#include "ap_commonvehsigprovider/gear_lever_information_c.h"
#include "eco/memset_c.h"

/// From Gearbox Control.
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{nu}
    ///
    AP_COMMONVEHSIGPROVIDER_GearInformation gearInformation;
    ///@unit{nu}
    ///
    AP_COMMONVEHSIGPROVIDER_GearLeverInformation gearLeverInformation;
} AP_COMMONVEHSIGPROVIDER_GearboxCtrlStatusPort;

inline AP_COMMONVEHSIGPROVIDER_GearboxCtrlStatusPort create_AP_COMMONVEHSIGPROVIDER_GearboxCtrlStatusPort(void)
{
  AP_COMMONVEHSIGPROVIDER_GearboxCtrlStatusPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.gearInformation = create_AP_COMMONVEHSIGPROVIDER_GearInformation();
  m.gearLeverInformation = create_AP_COMMONVEHSIGPROVIDER_GearLeverInformation();
  return m;
}

#endif // AP_COMMONVEHSIGPROVIDER_GEARBOX_CTRL_STATUS_PORT_C_H_
