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

#ifndef MF_MEMPARK_TRAJECTORY_META_DATA_C_H_
#define MF_MEMPARK_TRAJECTORY_META_DATA_C_H_

#include "mf_mempark/mem_park_date_t_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Meta data of the parking trajectory
typedef struct
{
    ///Date containing mainly day/month/year,
    ///as auxiliary info could include time (hour, minute, second)
    ///and time zone of the moment where the trajectory was saved.
    MF_MEMPARK_MemParkDate_t trajectorySaveDate;
    ///[Optional] Identifies the vehicle that saved the trajectory.
    ///See https://confluence.auto.continental.cloud/display/PLP/Slots+and+Trajectories+Interfaces
    ///Temporary datatype set, should be string!
    uint8 egoVehicle;
} MF_MEMPARK_TrajectoryMetaData;

inline MF_MEMPARK_TrajectoryMetaData create_MF_MEMPARK_TrajectoryMetaData(void)
{
  MF_MEMPARK_TrajectoryMetaData m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.trajectorySaveDate = create_MF_MEMPARK_MemParkDate_t();
  return m;
}

#endif // MF_MEMPARK_TRAJECTORY_META_DATA_C_H_