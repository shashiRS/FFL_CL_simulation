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

#ifndef MF_MEMPARK_PARKING_TRAJECTORY_H_
#define MF_MEMPARK_PARKING_TRAJECTORY_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "lsm_geoml/pose_pod.h"
#include "mf_mempark/trajectory_point.h"
#include "mf_mempark/trajectory_meta_data.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// Describes the 2D path followed by the vehicle during a parking maneuver.
  struct ParkingTrajectory
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    uint8 trajectoryID;
    ::lsm_geoml::Pose_POD startPose;
    ::lsm_geoml::Pose_POD endPose;
    uint8 numValidTrajPoints;
    TrajectoryPoint listOfPoints[200];
    uint8 slotID;
    TrajectoryMetaData metaData;
  };

  inline ::mf_mempark::ParkingTrajectory createParkingTrajectory()
  {
    ParkingTrajectory m;
    (void)::eco::memset(&m, 0U, sizeof(ParkingTrajectory));
    m.sSigHeader = ::eco::createSignalHeader();
    m.startPose = ::lsm_geoml::createPose_POD();
    m.endPose = ::lsm_geoml::createPose_POD();
    {
      const uint64 arraysize = (sizeof(m.listOfPoints) / sizeof(m.listOfPoints[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.listOfPoints[i] = createTrajectoryPoint();
      }
    }
    m.metaData = createTrajectoryMetaData();
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::ParkingTrajectory create_default()
  {
      return ::mf_mempark::createParkingTrajectory();
  }
}


#endif // MF_MEMPARK_PARKING_TRAJECTORY_H_
