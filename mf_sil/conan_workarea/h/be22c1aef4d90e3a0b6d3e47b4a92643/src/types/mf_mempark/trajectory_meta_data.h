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

#ifndef MF_MEMPARK_TRAJECTORY_META_DATA_H_
#define MF_MEMPARK_TRAJECTORY_META_DATA_H_

#include "mf_mempark/mem_park_date_t.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_mempark
{

  /// Meta data of the parking trajectory
  struct TrajectoryMetaData
  {
    ///Date containing mainly day/month/year,
    ///as auxiliary info could include time (hour, minute, second)
    ///and time zone of the moment where the trajectory was saved.
    MemParkDate_t trajectorySaveDate;
    ///[Optional] Identifies the vehicle that saved the trajectory.
    ///See https://confluence.auto.continental.cloud/display/PLP/Slots+and+Trajectories+Interfaces
    ///Temporary datatype set, should be string!
    uint8 egoVehicle;
  };

  inline ::mf_mempark::TrajectoryMetaData createTrajectoryMetaData()
  {
    TrajectoryMetaData m;
    (void)::eco::memset(&m, 0U, sizeof(TrajectoryMetaData));
    m.trajectorySaveDate = createMemParkDate_t();
    return m;
  }

} // namespace mf_mempark

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_mempark::TrajectoryMetaData create_default()
  {
      return ::mf_mempark::createTrajectoryMetaData();
  }
}


#endif // MF_MEMPARK_TRAJECTORY_META_DATA_H_
