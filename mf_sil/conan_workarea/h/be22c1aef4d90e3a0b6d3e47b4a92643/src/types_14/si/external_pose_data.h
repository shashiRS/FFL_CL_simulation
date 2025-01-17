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

#ifndef SI_EXTERNAL_POSE_DATA_H_
#define SI_EXTERNAL_POSE_DATA_H_

#include "lsm_geoml/pose_pod.h"
#include "Platform_Types.h"


namespace si
{

  struct ExternalPoseData
  {
    ::lsm_geoml::Pose_POD externalTargetPose_m{};
    float32 curvature_1pm{};
    uint16 relatedParkingBoxId{65535U};
    boolean isParkingPose{};
    boolean isForwardDriving{};
  };

} // namespace si

#endif // SI_EXTERNAL_POSE_DATA_H_
