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

#ifndef MF_HMIH_HMIGENERAL_H_
#define MF_HMIH_HMIGENERAL_H_

#include "ap_psm_app/remote_mode.h"
#include "Platform_Types.h"
#include "ap_psm_app/max_speed10_kphwarning.h"
#include "ap_common/driving_direction.h"
#include "ap_psm_app/garage_parking.h"
#include "mf_hmih/avgtype.h"
#include "ap_commonvehsigprovider/gear.h"
#include "ap_psm_app/hmimessage.h"
#include "ap_psm_app/ppcparking_mode.h"
#include "ap_psm_app/apfinish_type.h"
#include "mf_drvwarnsm/pdwsystem_state.h"
#include "mf_lvmd/lvmdsystem_status.h"
#include "mf_lvmd/lvmdwarning_trigger.h"
#include "mf_drvwarnsm/pdwshutdown_cause.h"
#include "mf_drvwarnsm/whpstate.h"
#include "mf_hmih/blind_spot_view_status.h"
#include "mf_hmih/slot_unreach_reason.h"


namespace mf_hmih
{

  /// Major screen control of HMI.
  struct HMIGeneral
  {
    ///@range{0,4}
    ///[Optional] Indicates the currently active mode of AP related to remote functionality
    ::ap_psm_app::RemoteMode remoteModeActive_nu{};
    ///Indicates, whether the Remote App is active (= is opened and is not breaked down)
    boolean remoteAppActive_nu{};
    ///True if the usage of the remote Control is authorized
    boolean remoteAppAuthorized_nu{};
    ///Indicates, that the remote control is coded for the current sw
    boolean remoteAppCoded_nu{};
    ///True if user selected Remote Key instead of in vehicle parking
    boolean remoteKeySelected_nu{};
    ///Indicates, whether Remote Key is possible in the vehicle
    boolean remoteKeyPoss_nu{};
    ///Indicates wether it is possible to continue the interrupted maneuver
    boolean continuePoss_nu{};
    ///True, if remote parking in is possible. (In this case, user can select it via menu of the remote parking app)
    boolean parkInPoss_nu{};
    ///True, if remote parking out is possible. (In this case, user can select it via menu of the remote parking app)
    boolean parkOutPoss_nu{};
    ///True, if remote maneuvering is possible. (In this case, user can select it via menu of the remote parking app)
    boolean remManPoss_nu{};
    ///Indicates wether it is possible to undo the interrupted maneuver
    boolean undoPoss_nu{};
    ///True, if the vehicle can stream a surround-view view to the remote parking app
    boolean svPoss_nu{};
    ///[for Remote Maneuvering]True, if it is possible to move the ego vehicle in forward direction -> Forward-Button is active in REM_MAN-Screen
    boolean btnForwardPoss_nu{};
    ///[for Remote Maneuvering]True, if it is possible to move the ego vehicle in backward direction -> Backward-Button is active in REM_MAN-Screen
    boolean btnBackwardPoss_nu{};
    ///Indicates, if Fully Automated Parking is possible (shows the corespondent button)
    boolean btnFullyAutomParkingPoss_nu{};
    ///Indicates, if Semi Automated Parking is possible (shows the corespondent button)
    boolean btnSemiAutomParkingPoss_nu{};
    ///Indicates, that a remote garage door opener is available and configured (true, when GARAGE_OPENER_STATUS_NU == GOS_CONFIGURED)
    boolean garageOpenerAvail_nu{};
    ///@unit{Percent}
    ///@range{0,102}
    ///Distance to next stopping point during parking maneuver (could be shown in a filling bar)
    uint8 distanceToStop_perc{};
    ///@range{0,2}
    ///For manual gear box, warning maximum speed limit : 10KPH
    ::ap_psm_app::MaxSpeed10KPHwarning maxSpeed10KPHwarning_nu{};
    ///@range{0,3}
    ///Vehicle direction for the current maneuver part
    ::ap_common::DrivingDirection drivingDirection_nu{};
    ///@range{0,6}
    ///0: set, when either GARAGE_PARKING_CODED_NU==false or when no garage has been detected 1: set, when a garage scan is active in the front of the vehicle 2: set, when a garage scan is active in the back of the vehicle 3: set, wehen a forward parking out situation has been detected . 4: set, when a backward parking out situation has been detected 5: set, when a garage has been detected in the front of the vehicle 6: set, when a garage has been detected in the back of the vehicle
    ::ap_psm_app::GarageParking garageParking_nu{};
    ///@range{0,15}
    ///Shows the simplified situation during Automated Vehicle Guidance to enable HMI to inform driver of maneuver.
    AVGType avgType_nu{};
    ///@range{0,15}
    ///Current gear of the ego vehicle. This information is displayed in automated vehicle guidance (AVG) mode.
    ::ap_commonvehsigprovider::Gear currentGear_nu{};
    ///@range{0,58}
    ///general information for the User. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    ::ap_psm_app::HMIMessage generalUserInformation_nu{};
    ///@range{0,11}
    ///current active parking mode. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    ::ap_psm_app::PPCParkingMode ppcParkingMode_nu{};
    ///@range{0,3}
    ///type of the finish event. Extended User interface. Currently NOT used. Can be used to get rid of the visualization ports
    ::ap_psm_app::APFinishType finishType_nu{};
    ///@range{0,5}
    ///PDw system state info
    ::mf_drvwarnsm::PDWSystemState pdcSystemState_nu{};
    ///@range{0,4}
    ///LVMD system state info
    ::mf_lvmd::LVMDSystemStatus lvmdSystemState_nu{};
    ///@range{0,2}
    ///LVMD warning type
    ::mf_lvmd::LVMDWarningTrigger lvmdWarningType_nu{};
    ///@range{0,3}
    ///PDW shutdown cause in case PDW system state is off
    ::mf_drvwarnsm::PDWShutdownCause pdcShutdownCause_nu{};
    ///@range{0,4}
    ///WHP state info
    ::mf_drvwarnsm::WHPState whpState_nu{};
    ///WHP display request
    boolean whpDisplayReq_nu{};
    ///describs if it is possible to switch the input device
    boolean apSwitchInputDevicePoss_nu{};
    boolean memoryParkingPoss_nu{};
    ///Indicates if the back button should be displayed. Purpose of the button is to allow the user to go back to top view
    ///from the 3D view that is displayed once a slot was selected by the user.
    boolean displayBackButton_nu{};
    BlindSpotViewStatus blindSpotViewStatus_nu{};
    ///Describes the reason why the selected slot is not reachable, when the user selects an unreachable pose.
    SlotUnreachReason slotUnreachReason_nu{};
    ///Bitarray indicating which of the memory slots are occupied (bit X = 1 means slot X contains a memorized user defined pose).
    uint8 memorySlotsStatus_nu{};
    ///Indicates the memory slot id that stores the redetected memorized pose.
    uint8 redetectedPoseMemSlotId_nu{255U};
    ///Indicates if the adjustments buttons are disabled or if movement in the selected direction is still possibile(Bit X=1 means movement button X is available)
    uint8 adjustmentButtons_nu{};
  };

} // namespace mf_hmih

#endif // MF_HMIH_HMIGENERAL_H_
