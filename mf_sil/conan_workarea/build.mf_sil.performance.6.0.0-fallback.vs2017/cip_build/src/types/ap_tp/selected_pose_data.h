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

#ifndef AP_TP_SELECTED_POSE_DATA_H_
#define AP_TP_SELECTED_POSE_DATA_H_

#include "ap_tp/pose_selection_status.h"
#include "ap_tp/pose_reached_status.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_tp
{

  /// 
  struct SelectedPoseData
  {
    ///@range{0,3}
    ///@unit{enum PoseSelectionStatus}
    ///Indicates the preselection of poses done by planner @min: 0 @max: 3 @unit: enum PoseSelectionStatus  @values: enum { _PSS_NO_SELECTION=0,_PSS_PLANNER_PRESELECTION=1,_PSS_DRIVER_SELECTION=2,_MAX_NUM_POSE_SEL_STATUS_TYPES=3 }
    PoseSelectionStatus selectionStatus;
    ///@range{0,3}
    ///@unit{enum PoseReachedStatus}
    ///Indicates wether the selected pose is reached by the ego vehicle within the required tolerances @min: 0 @max: 3 @unit: enum PoseReachedStatus @values: enum { _NO_TP_REACHED_STATUS=0,_TP_REACHED=1,_TP_REACHED_FALLBACK=2,_TP_NOT_REACHED=3,_MAX_NUM_POSE_REACHED_STATUS_TYPES=4 }
    PoseReachedStatus reachedStatus;
    ///@range{0,64}
    ///@unit{m}
    ///None @min: 0 @max: 64 @unit: m
    float32 distanceToStart_m;
  };

  inline ::ap_tp::SelectedPoseData createSelectedPoseData()
  {
    SelectedPoseData m;
    (void)::eco::memset(&m, 0U, sizeof(SelectedPoseData));
    return m;
  }

} // namespace ap_tp

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_tp::SelectedPoseData create_default()
  {
      return ::ap_tp::createSelectedPoseData();
  }
}


#endif // AP_TP_SELECTED_POSE_DATA_H_