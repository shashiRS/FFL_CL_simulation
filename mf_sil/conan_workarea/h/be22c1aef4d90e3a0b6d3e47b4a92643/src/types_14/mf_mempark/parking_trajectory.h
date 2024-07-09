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

#ifndef MF_MEMPARK_PARKING_TRAJECTORY_H_
#define MF_MEMPARK_PARKING_TRAJECTORY_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "lsm_geoml/pose_pod.h"
#include "mf_mempark/trajectory_point.h"
#include "mf_mempark/trajectory_meta_data.h"


namespace mf_mempark
{

  /// Describes the 2D path followed by the vehicle during a parking maneuver.
  struct ParkingTrajectory
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    uint8 trajectoryID{};
    ::lsm_geoml::Pose_POD startPose{};
    ::lsm_geoml::Pose_POD endPose{};
    uint8 numValidTrajPoints{};
    TrajectoryPoint listOfPoints[200]{};
    uint8 slotID{};
    TrajectoryMetaData metaData{};
  };

} // namespace mf_mempark

#endif // MF_MEMPARK_PARKING_TRAJECTORY_H_
