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

#ifndef AP_TP_TARGET_POSES_PORT_H_
#define AP_TP_TARGET_POSES_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_tp/planning_fail_reason.h"
#include "ap_tp/selected_pose_data.h"
#include "ap_tp/target_pose.h"


namespace ap_tp
{

  /// 
  struct TargetPosesPort
  {
    ///@unit{eco.AlgoInterfaceVersionNumber}
    ///
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ///@unit{eco.SignalHeader}
    ///
    ::eco::SignalHeader sSigHeader{};
    ///@unit{Count}
    ///indicates number of coordinate resets performed by the planner @min: 0 @max: 0 @unit: Count
    uint8 resetCounter{};
    ///@unit{enum PlanningFailReason}
    ///@range{0,4}
    ///Reason for failing from TPD @min: 0 @max: 4 @unit: enum PlanningFailReason @values: enum { _PFR_NONE=0,_PFR_TARGET_POSE_LOST=1,_PFR_PARKING_BOX_LOST=2,_PFR_INPUT_CORRUPTED=3,_PFR_REPLAN_FAIL=4,_MAX_NUM_PLANNING_FAIL_TYPES=5 }
    PlanningFailReason failReason{};
    ///@unit{boolean}
    ///@range{0,1}
    ///None @min: 0 @max: 1 @unit: boolean
    boolean anyPathFound{};
    ///@unit{nu}
    ///Information related to the selected Target Pose @min: 0 @max: 0 @unit: nu
    SelectedPoseData selectedPoseData{};
    ///@unit{Count}
    ///Number of poses that are valid (does not need to be reachable) @min: 0 @max: AP_Common.AP_G_MAX_NUM_TARGET_POSES_NU @unit: Count
    uint8 numValidPoses{};
    ///@unit{nu}
    ///All information related to a possible target pose @min: 0 @max: 0 @unit: nu
    TargetPose targetPoses[8]{};
  };

} // namespace ap_tp

#endif // AP_TP_TARGET_POSES_PORT_H_
