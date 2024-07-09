// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

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

#ifndef MF_MANAGER_MFMPLANNED_TRAJ_TYPE_H_
#define MF_MANAGER_MFMPLANNED_TRAJ_TYPE_H_

#include "Platform_Types.h"

namespace mf_manager
{
  enum class MFMPlannedTrajType : uint8
  {
      MFM_PERP_PARK_IN_TRAJ = 0U,
      MFM_PAR_PARK_IN_TRAJ = 1U,
      MFM_PERP_PARK_OUT_TRAJ = 2U,
      MFM_PAR_PARK_OUT_TRAJ = 3U,
      MFM_REMOTE_MAN_TRAJ = 4U,
      MFM_UNDO_TRAJ = 5U,
      MFM_MAX_NUM_PLANNED_TRAJ_TYPES = 6U,
  };
} // namespace mf_manager
#endif // MF_MANAGER_MFMPLANNED_TRAJ_TYPE_H_