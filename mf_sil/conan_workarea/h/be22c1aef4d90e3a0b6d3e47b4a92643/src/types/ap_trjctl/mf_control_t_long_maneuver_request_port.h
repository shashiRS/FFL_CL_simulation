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

#ifndef AP_TRJCTL_MF_CONTROL_T_LONG_MANEUVER_REQUEST_PORT_H_
#define AP_TRJCTL_MF_CONTROL_T_LONG_MANEUVER_REQUEST_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_trjctl/mf_control_t_long_ctrl_req.h"
#include "ap_trjctl/moco_t_long_lims.h"
#include "ap_trjctl/moco_te_fun_mode.h"
#include "ap_trjctl/moco_te_dyn_mode.h"
#include "ap_trjctl/moco_te_mot_req.h"
#include "Platform_Types.h"
#include "ap_trjctl/moco_te_driving_dir_req.h"
#include "eco/memset.h"


namespace ap_trjctl
{

  /// Longitudinal request to underlaid Trajectory Tracking Controller (TRATCO). (MoCo variant only)
  struct MF_CONTROL_t_LongManeuverRequestPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///Longitudinal control request (control mode and control reference values)
    MF_CONTROL_t_LongCtrlReq ctrlReq;
    ///Longitudinal limitations of acceleration and jerk.
    MOCO_t_LongLims lims;
    ///@range{0,9}
    ///Function request identifier. (e.g. MOCO_FUN_MODE_MANEUVER)
    MOCO_te_FunMode funMode;
    ///@range{0,2}
    ///Dynamic mode. (Low, Medium, High)
    MOCO_te_DynMode dynMode;
    ///@range{0,2}
    ///Motion request (SUPPRESS_STANDSTILL=0, STANDSTILL=1, DRIVEOF = 2)
    MOCO_te_MotReq motReq;
    ///Control command to activate/deactivate the longitudinal control. (true==activate; false==deactivate)
    uint8 activateCtrl;
    ///Request to stop the vehicle in emergency cases.
    uint8 fullBrakingReq;
    ///@range{0,2}
    ///Requested driving direction for longitudinal control. (NO_REQ = 0, FORWARD = 1, BACKWARD = 2)
    MOCO_te_DrivingDirReq drivingDirReq;
  };

  inline ::ap_trjctl::MF_CONTROL_t_LongManeuverRequestPort createMF_CONTROL_t_LongManeuverRequestPort()
  {
    MF_CONTROL_t_LongManeuverRequestPort m;
    (void)::eco::memset(&m, 0U, sizeof(MF_CONTROL_t_LongManeuverRequestPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.ctrlReq = createMF_CONTROL_t_LongCtrlReq();
    m.lims = createMOCO_t_LongLims();
    m.dynMode = ::ap_trjctl::MOCO_te_DynMode::MOCO_DYN_MODE_MEDIUM;
    return m;
  }

} // namespace ap_trjctl

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_trjctl::MF_CONTROL_t_LongManeuverRequestPort create_default()
  {
      return ::ap_trjctl::createMF_CONTROL_t_LongManeuverRequestPort();
  }
}


#endif // AP_TRJCTL_MF_CONTROL_T_LONG_MANEUVER_REQUEST_PORT_H_
