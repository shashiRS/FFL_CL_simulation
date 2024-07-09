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

#ifndef AP_COMMONVEHSIGPROVIDER_ENGINE_CTRL_STATUS_PORT_C_H_
#define AP_COMMONVEHSIGPROVIDER_ENGINE_CTRL_STATUS_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_commonvehsigprovider/start_stop_status_c.h"
#include "eco/memset_c.h"

/// Signals from Engine ECU
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{Percentage}
    ///@range{0,101.6}
    ///Throttle position in percent
    float32 throttlePos_perc;
    ///@unit{Percentage per second}
    ///Throttle gradient is calculated by two 4 ms scanned throttle positions
    float32 throttleGradient_percps;
    ///@unit{torque in Nm}
    ///Torquq at axle in Nm
    float32 axleTorque_nm;
    ///Engine is running
    boolean engineOn_nu;
    ///@range{0,3}
    ///Status of start stop function should be in status N/A or "no clearance" to prevent turning off the engine while parking
    AP_COMMONVEHSIGPROVIDER_StartStopStatus startStopStatus_nu;
    ///Remote start possible
    boolean remoteStartPossible_nu;
    ///Qualifier-Bit of Throttle position
    boolean throttlePos_QF_nu;
    ///Qualifier-Bit Throttle position
    boolean throttleGradient_QF_nu;
} AP_COMMONVEHSIGPROVIDER_EngineCtrlStatusPort;

inline AP_COMMONVEHSIGPROVIDER_EngineCtrlStatusPort create_AP_COMMONVEHSIGPROVIDER_EngineCtrlStatusPort(void)
{
  AP_COMMONVEHSIGPROVIDER_EngineCtrlStatusPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_COMMONVEHSIGPROVIDER_ENGINE_CTRL_STATUS_PORT_C_H_
