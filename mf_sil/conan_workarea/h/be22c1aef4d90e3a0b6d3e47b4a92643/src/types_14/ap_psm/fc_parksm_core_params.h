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

#ifndef AP_PSM_FC_PARKSM_CORE_PARAMS_H_
#define AP_PSM_FC_PARKSM_CORE_PARAMS_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"


namespace ap_psm
{

  /// FC_PARKSM_Core Parameters
  struct FC_PARKSM_Core_Params
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///The state machine has to request a reset of the map after AP_S_RESET_MAP_DIST_M meter is driven.
    float32 AP_SC_RESET_MAP_DIST_M{};
    ///Indicates if reverse assist is enabled (coded).
    boolean AP_SC_REV_ASSIST_CODED{};
  };

} // namespace ap_psm

#endif // AP_PSM_FC_PARKSM_CORE_PARAMS_H_
