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

#ifndef MF_HMIH_PARKING_TARGET_POSES_H_
#define MF_HMIH_PARKING_TARGET_POSES_H_

#include "Platform_Types.h"
#include "mf_hmih/parking_target_pose.h"


namespace mf_hmih
{

  /// Target poses information for Visu
  struct ParkingTargetPoses
  {
    ///Number of poses that are valid (does not need to be reachable)
    uint8 numValidParkingPoses_nu{255U};
    ///All information related to a possible target pose
    ParkingTargetPose parkingPoses[20]{};
  };

} // namespace mf_hmih

#endif // MF_HMIH_PARKING_TARGET_POSES_H_
