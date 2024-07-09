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

#ifndef AP_LADMC_LA_DMCSTATUS_PORT_C_H_
#define AP_LADMC_LA_DMCSTATUS_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_ladmc/la_dmcsystem_state_c.h"
#include "eco/memset_c.h"

/// Signals for control logic communication with other Funtionc Components. (e.g. Trajectory Control)
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{Nm}
    ///@range{-15,15}
    ///Detected Steering Torque by driver
    float32 handSteeringTorque_Nm;
    ///Qualifier-Bit of handSteeringTorque
    boolean handSteeringTorque_QF_nu;
    ///@range{0,7}
    ///System status/Availability of the lateral dynamic motion control and the underlaid actuator EPS. Used for communication between LaDMC and EPS as well as LaDMC and StateMachine.
    AP_LADMC_LaDMCSystemState laDMCSystemState_nu;
    ///Flag about detected driver intervention. (e.g. steer wheel hand torque detected)
    boolean driverIntervention_nu;
} AP_LADMC_LaDMCStatusPort;

inline AP_LADMC_LaDMCStatusPort create_AP_LADMC_LaDMCStatusPort(void)
{
  AP_LADMC_LaDMCStatusPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_LADMC_LA_DMCSTATUS_PORT_C_H_
