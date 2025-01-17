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

#ifndef AP_PSM_APP_OVERRIDE_LSCAPORT_H_
#define AP_PSM_APP_OVERRIDE_LSCAPORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "ap_common/driving_direction.h"


namespace ap_psm_app
{

  struct OverrideLSCAPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber{};
    ::eco::SignalHeader sSigHeader{};
    ///@range{0,1}
    ///Request for distance to STOP from AP and LSCA in order to ignore the object inside the given distance
    float32 ignoreDistanceLSCA_m{};
    ///Request for direction of distance to ignore the object in this direction
    ::ap_common::DrivingDirection ignoreDirection_nu{};
  };

} // namespace ap_psm_app

#endif // AP_PSM_APP_OVERRIDE_LSCAPORT_H_
