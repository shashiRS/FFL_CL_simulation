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

#ifndef AP_PSM_APP_CTRL_COMMAND_PORT_H_
#define AP_PSM_APP_CTRL_COMMAND_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_psm_app/std_request.h"
#include "ap_psm_app/ppcparking_mode.h"
#include "ap_psm_app/degrade_type.h"
#include "Platform_Types.h"
#include "ap_psm_app/motion_control_request_type.h"
#include "ap_psm_app/driver_in_out_request_type.h"


namespace ap_psm_app
{

  struct CtrlCommandPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///the std request to the ap psm core
    StdRequest stdRequest_nu{};
    ///inform the psm core of the current parking mode
    PPCParkingMode ppcParkingMode_nu{};
    ///in case of function degradation is requeseted the type of the degradation has to be specified
    DegradeType degradeType_nu{};
    ///id of the selected target pose
    uint8 selectedTargetPoseId_nu{};
    ///request emergency brake of core parksm (if this flag is set the stop request will be send as emergency brake request to the controller)
    boolean emergencyBrake_nu{};
    ///Id of the current memory parking slot selected by the user
    uint8 currentSelectedMpSlotId{};
    ///request to delete
    boolean deleteMemorizedParkingData{};
    ///store request
    boolean storeMemorizedParkingData{};
    ///Type of requests to the Motion Controller
    MotionControlRequestType motionControlRequestType_nu{};
    ///Type of requests based on driver Inside or outside
    DriverInOutRequestType driverInOutRequestType_nu{};
  };

} // namespace ap_psm_app

#endif // AP_PSM_APP_CTRL_COMMAND_PORT_H_
