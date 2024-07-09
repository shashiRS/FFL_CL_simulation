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

#ifndef VC_SELECTED_TARGET_POSE_C_H_
#define VC_SELECTED_TARGET_POSE_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// This struct will be provided by visu component actually. But because visu is
/// not yet a GitHub component, we made a compromise and declared it here.
/// The port will be consumed by HMI Handler.
typedef struct
{
    uint8 selected_pose_id;
    boolean switch_pose_orientation;
} VC_SelectedTargetPose;

inline VC_SelectedTargetPose create_VC_SelectedTargetPose(void)
{
  VC_SelectedTargetPose m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // VC_SELECTED_TARGET_POSE_C_H_
