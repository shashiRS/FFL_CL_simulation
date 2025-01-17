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

#ifndef AP_PSM_TRAJ_CTRL_REQUEST_PORT_H_
#define AP_PSM_TRAJ_CTRL_REQUEST_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_psm/driving_mode_trc.h"
#include "Platform_Types.h"
#include "ap_psm/motion_control_request_type.h"
#include "ap_psm/driver_in_out_request_type.h"


namespace ap_psm
{

  /// Control commands from Parking State Machine to Trajectory Control.
  struct TrajCtrlRequestPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@range{0,3}
    ///Define the requested driving mode. Only in the mode "FOLLOW_TRAJ" the requested trajectory will be used as input for the trajectory control.
    DrivingModeTRC drivingModeReq_nu{};
    ///Control command for the activation/deactivation of the trajectory control (automatic vehicle guidance).
    boolean trajCtrlActive_nu{};
    ///@unit{boolean}
    ///Triggers an emergency braking intervention.
    boolean emergencyBrakeRequest{};
    ///Type of requests to the Motion Controller .
    MotionControlRequestType MotionControlRequestType_nu{};
    ///Type of requests based on driver Inside or outside.
    DriverInOutRequestType driverInOutRequestType_nu{};
  };

} // namespace ap_psm

#endif // AP_PSM_TRAJ_CTRL_REQUEST_PORT_H_
