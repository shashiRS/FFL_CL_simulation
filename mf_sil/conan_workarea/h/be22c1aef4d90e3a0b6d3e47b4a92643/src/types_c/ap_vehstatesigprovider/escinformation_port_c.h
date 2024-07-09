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

#ifndef AP_VEHSTATESIGPROVIDER_ESCINFORMATION_PORT_C_H_
#define AP_VEHSTATESIGPROVIDER_ESCINFORMATION_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_vehstatesigprovider/tcsstate_c.h"
#include "ap_vehstatesigprovider/escstate_c.h"
#include "ap_vehstatesigprovider/absstate_c.h"
#include "ap_vehstatesigprovider/ebdstate_c.h"
#include "eco/memset_c.h"

/// Signals from ESC
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{bar per second}
    ///@range{0,2500}
    ///Brake pressure gradient of driver"s brake pressure
    float32 brakePressureGradient_barps;
    ///@unit{bar}
    ///@range{-30,276.6}
    ///Brake pressure by driver
    float32 brakePressureDriver_bar;
    ///@range{0,1}
    ///Traction Control System active
    AP_VEHSTATESIGPROVIDER_TCSState tcsState_nu;
    ///@range{0,1}
    ///ESC active
    AP_VEHSTATESIGPROVIDER_ESCState escState_nu;
    ///@range{0,1}
    ///ABS active
    AP_VEHSTATESIGPROVIDER_ABSState absState_nu;
    ///@range{0,1}
    ///Electronic Brakeforce Distribution active
    AP_VEHSTATESIGPROVIDER_EBDState ebdState_nu;
} AP_VEHSTATESIGPROVIDER_ESCInformationPort;

inline AP_VEHSTATESIGPROVIDER_ESCInformationPort create_AP_VEHSTATESIGPROVIDER_ESCInformationPort(void)
{
  AP_VEHSTATESIGPROVIDER_ESCInformationPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_VEHSTATESIGPROVIDER_ESCINFORMATION_PORT_C_H_