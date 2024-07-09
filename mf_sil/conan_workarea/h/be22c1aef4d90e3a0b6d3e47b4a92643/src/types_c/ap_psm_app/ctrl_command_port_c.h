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

#ifndef AP_PSM_APP_CTRL_COMMAND_PORT_C_H_
#define AP_PSM_APP_CTRL_COMMAND_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "ap_psm_app/std_request_c.h"
#include "ap_psm_app/ppcparking_mode_c.h"
#include "ap_psm_app/degrade_type_c.h"
#include "Platform_Types.h"
#include "ap_psm_app/motion_control_request_type_c.h"
#include "ap_psm_app/driver_in_out_request_type_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///the std request to the ap psm core
    AP_PSM_APP_StdRequest stdRequest_nu;
    ///inform the psm core of the current parking mode
    AP_PSM_APP_PPCParkingMode ppcParkingMode_nu;
    ///in case of function degradation is requeseted the type of the degradation has to be specified
    AP_PSM_APP_DegradeType degradeType_nu;
    ///id of the selected target pose
    uint8 selectedTargetPoseId_nu;
    ///request emergency brake of core parksm (if this flag is set the stop request will be send as emergency brake request to the controller)
    boolean emergencyBrake_nu;
    ///Id of the current memory parking slot selected by the user
    uint8 currentSelectedMpSlotId;
    ///request to delete
    boolean deleteMemorizedParkingData;
    ///store request
    boolean storeMemorizedParkingData;
    ///Type of requests to the Motion Controller
    AP_PSM_APP_MotionControlRequestType motionControlRequestType_nu;
    ///Type of requests based on driver Inside or outside
    AP_PSM_APP_DriverInOutRequestType driverInOutRequestType_nu;
} AP_PSM_APP_CtrlCommandPort;

inline AP_PSM_APP_CtrlCommandPort create_AP_PSM_APP_CtrlCommandPort(void)
{
  AP_PSM_APP_CtrlCommandPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // AP_PSM_APP_CTRL_COMMAND_PORT_C_H_
