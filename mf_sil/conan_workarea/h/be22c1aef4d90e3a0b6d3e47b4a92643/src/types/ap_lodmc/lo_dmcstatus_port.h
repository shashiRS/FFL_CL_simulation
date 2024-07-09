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

#ifndef AP_LODMC_LO_DMCSTATUS_PORT_H_
#define AP_LODMC_LO_DMCSTATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_lodmc/slope_accuracy.h"
#include "ap_lodmc/lo_dmcsystem_state.h"
#include "ap_lodmc/maneuvering_finished.h"
#include "ap_lodmc/longitudinal_control_active_status.h"
#include "eco/memset.h"


namespace ap_lodmc
{

  /// Signals for control logic communication with other Funtionc Components. (e.g. Trajectory Control)
  struct LoDMCStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@unit{%}
    ///@range{-30,30}
    ///Current estimation of dynamic slope.
    float32 dynamicSlope_perc;
    ///@range{0,3}
    ///Accurcay of current estimation of dynamic slope.
    SlopeAccuracy dynamicSlopeAccuracy_nu;
    ///@range{0,15}
    ///System status and availability of the longitudinal dynamic motion control and the underlaid actuators.
    LoDMCSystemState loDMCSystemState_nu;
    ///Information that the requested maneuvering is in progress or finished by the LoDMC. (e.g. requested DistanceToStopReq_nu reached and vehicle in stand still)
    maneuveringFinished maneuveringFinished_nu;
    longitudinalControlActiveStatus longitudinalControlActiveStatus_nu;
    ///Status defines whether the vehicle standstill is hold by the LoDMC. (e.g. using the hydraulic brake)
    boolean standstillHoldCur_nu;
    ///Status defines whether the vehicle standstill is secured by the LoDMC. (e.g. closed electric park brake)
    boolean standstillSecureCur_nu;
  };

  inline ::ap_lodmc::LoDMCStatusPort createLoDMCStatusPort()
  {
    LoDMCStatusPort m;
    (void)::eco::memset(&m, 0U, sizeof(LoDMCStatusPort));
    m.sSigHeader = ::eco::createSignalHeader();
    return m;
  }

} // namespace ap_lodmc

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_lodmc::LoDMCStatusPort create_default()
  {
      return ::ap_lodmc::createLoDMCStatusPort();
  }
}


#endif // AP_LODMC_LO_DMCSTATUS_PORT_H_