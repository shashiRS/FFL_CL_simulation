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

#ifndef VC_SELECTED_TARGET_POSE_H_
#define VC_SELECTED_TARGET_POSE_H_

#include "Platform_Types.h"


namespace vc
{

  /// This struct will be provided by visu component actually. But because visu is
  /// not yet a GitHub component, we made a compromise and declared it here.
  /// The port will be consumed by HMI Handler.
  struct SelectedTargetPose
  {
    uint8 selected_pose_id{};
    boolean switch_pose_orientation{};
  };

} // namespace vc

#endif // VC_SELECTED_TARGET_POSE_H_
