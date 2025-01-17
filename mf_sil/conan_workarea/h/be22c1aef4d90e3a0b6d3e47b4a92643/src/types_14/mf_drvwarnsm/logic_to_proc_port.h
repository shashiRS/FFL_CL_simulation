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

#ifndef MF_DRVWARNSM_LOGIC_TO_PROC_PORT_H_
#define MF_DRVWARNSM_LOGIC_TO_PROC_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "mf_drvwarnsm/whl_warning_type.h"
#include "Platform_Types.h"
#include "mf_drvwarnsm/drv_tube_display_req.h"


namespace mf_drvwarnsm
{

  /// Signals from DrvWarn Logic to DrvWarn Functions Processing
  struct LogicToProcPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@range{0,2}
    ///Wheel warning suppression level for each wheel. Positions front left: 0, front right: 1, rear left: 2; rear right: 3, to be defined in an enum.
    WhlWarningType whlWarningType_nu[4]{};
    ///Reset flags for each side of the car. Positions front: 0, rear: 1, left: 2; right: 3, to be defined in an enum.
    boolean sideResetReq_nu[4]{};
    ///@range{0,2}
    ///DrvWarn logic determines the side where the driving tube should be displayed and informs PDW processing about it.
    DrvTubeDisplayReq drvTubeDisplayReq_nu{};
  };

} // namespace mf_drvwarnsm

#endif // MF_DRVWARNSM_LOGIC_TO_PROC_PORT_H_
