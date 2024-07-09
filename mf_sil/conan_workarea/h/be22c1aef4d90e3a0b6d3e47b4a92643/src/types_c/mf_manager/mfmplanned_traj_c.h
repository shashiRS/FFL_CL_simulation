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

#ifndef MF_MANAGER_MFMPLANNED_TRAJ_C_H_
#define MF_MANAGER_MFMPLANNED_TRAJ_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    float32 xTrajRAReq_m;
    float32 yTrajRAReq_m;
    float32 yawReq_rad;
    float32 crvRAReq_1pm;
    float32 distanceToStopReq_m;
    float32 velocityLimitReq_mps;
} MF_MANAGER_MFMPlannedTraj;

inline MF_MANAGER_MFMPlannedTraj create_MF_MANAGER_MFMPlannedTraj(void)
{
  MF_MANAGER_MFMPlannedTraj m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_MANAGER_MFMPLANNED_TRAJ_C_H_
