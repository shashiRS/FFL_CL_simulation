// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef AP_TRJCTL_LO_DMCCTRL_REQUEST_PORT_H_
#define AP_TRJCTL_LO_DMCCTRL_REQUEST_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_trjctl/lo_dmcctrl_request_source_type.h"
#include "ap_trjctl/lo_dmcctrl_request_interface_type.h"
#include "ap_trjctl/lo_dmcctrl_request_type.h"
#include "ap_trjctl/lo_dmchold_request_type.h"


namespace ap_trjctl
{

  /// Request to underlaid Longitudinal Dynamic Motion Control (LoDMC).
  struct LoDMCCtrlRequestPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@unit{m}
    ///@range{0,30}
    ///Value for the remaining distance to stop the vehicle at rear axle center. (*used as reference value in case of distance control)
    float32 distanceToStopReq_m{};
    ///@unit{m/s}
    ///@range{0,5}
    ///Value for the velocity limit / target vehicle velocity. (*used as reference value in case of velocity control; *used as limit for the vehicle velocity in case of distance and velocity control)
    float32 velocityLimitReq_mps{};
    ///@unit{m/s2}
    ///@range{-10,10}
    ///Reference / Target value for acceleration control.
    float32 accelerationReq_mps2{};
    ///@range{0,7}
    ///Source of the LoDMC control request
    LoDMCCtrlRequestSourceType loDMCCtrlRequestSource_nu{};
    ///@range{0,7}
    ///Type of the LoDMC control request: Used to select control signal of loDMCCtrlRequestPort (distance, acceleration, torque,...) in receiving component.
    LoDMCCtrlRequestInterfaceType loDMCCtrlRequestInterface_nu{::ap_trjctl::LoDMCCtrlRequestInterfaceType::MAX_NUM_LODMC_REQUEST_INTERFACE_TYPE};
    ///@range{0,4}
    ///Control command for type of longitudinal dynamic motion control request.
    LoDMCCtrlRequestType loDMCCtrlRequest_nu{};
    ///@range{0,2}
    ///Request to comfortably stop the vehicle in case of any vehicle movement and to hold the vehicle in standstill.
    LoDMCHoldRequestType holdReq_nu{};
    ///Request to stop the vehicle in emergency cases.
    boolean emergencyHoldReq_nu{};
    ///Request to secure the vehicle standstill. (e.g. close electric parking brake)
    boolean secureReq_nu{};
    ///Requested driving direction for longitudinal dynamic motion control. (true == forward; false == backward)
    boolean drivingForwardReq_nu{};
    ///Indicator for a trajectory reset. Will be set "true" in case of a reset.
    boolean trajectoryReset_nu{};
  };

} // namespace ap_trjctl

#endif // AP_TRJCTL_LO_DMCCTRL_REQUEST_PORT_H_
