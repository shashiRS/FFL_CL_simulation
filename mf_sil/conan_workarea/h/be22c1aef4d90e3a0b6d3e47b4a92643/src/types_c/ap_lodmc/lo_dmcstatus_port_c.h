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

#ifndef AP_LODMC_LO_DMCSTATUS_PORT_C_H_
#define AP_LODMC_LO_DMCSTATUS_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "ap_lodmc/slope_accuracy_c.h"
#include "ap_lodmc/lo_dmcsystem_state_c.h"
#include "ap_lodmc/maneuvering_finished_c.h"
#include "ap_lodmc/longitudinal_control_active_status_c.h"
#include "eco/memset_c.h"

/// Signals for control logic communication with other Funtionc Components. (e.g. Trajectory Control)
typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@unit{%}
    ///@range{-30,30}
    ///Current estimation of dynamic slope.
    float32 dynamicSlope_perc;
    ///@range{0,3}
    ///Accurcay of current estimation of dynamic slope.
    AP_LODMC_SlopeAccuracy dynamicSlopeAccuracy_nu;
    ///@range{0,15}
    ///System status and availability of the longitudinal dynamic motion control and the underlaid actuators.
    AP_LODMC_LoDMCSystemState loDMCSystemState_nu;
    ///Information that the requested maneuvering is in progress or finished by the LoDMC. (e.g. requested DistanceToStopReq_nu reached and vehicle in stand still)
    AP_LODMC_maneuveringFinished maneuveringFinished_nu;
    AP_LODMC_longitudinalControlActiveStatus longitudinalControlActiveStatus_nu;
    ///Status defines whether the vehicle standstill is hold by the LoDMC. (e.g. using the hydraulic brake)
    boolean standstillHoldCur_nu;
    ///Status defines whether the vehicle standstill is secured by the LoDMC. (e.g. closed electric park brake)
    boolean standstillSecureCur_nu;
} AP_LODMC_LoDMCStatusPort;

inline AP_LODMC_LoDMCStatusPort create_AP_LODMC_LoDMCStatusPort(void)
{
  AP_LODMC_LoDMCStatusPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_LODMC_LO_DMCSTATUS_PORT_C_H_
