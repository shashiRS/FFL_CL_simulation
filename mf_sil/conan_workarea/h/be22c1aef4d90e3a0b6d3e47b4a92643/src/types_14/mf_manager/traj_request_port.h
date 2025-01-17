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

#ifndef MF_MANAGER_TRAJ_REQUEST_PORT_H_
#define MF_MANAGER_TRAJ_REQUEST_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_manager/mfmplanned_traj.h"
#include "mf_manager/mfmplanned_traj_type.h"
#include "Platform_Types.h"
#include "mf_manager/mfmdriving_resistance.h"


namespace mf_manager
{

  struct TrajRequestPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    MFMPlannedTraj plannedTraj[20]{};
    MFMPlannedTrajType trajType_nu{::mf_manager::MFMPlannedTrajType::MFM_REMOTE_MAN_TRAJ};
    uint8 numValidCtrlPoints_nu{};
    boolean drivingForwardReq_nu{};
    boolean trajValid_nu{};
    boolean newSegmentStarted_nu{};
    boolean isLastSegment_nu{};
    uint8 stepInTrajAfterIdx_nu{255U};
    MFMDrivingResistance drivingResistance[4]{};
  };

} // namespace mf_manager

#endif // MF_MANAGER_TRAJ_REQUEST_PORT_H_
