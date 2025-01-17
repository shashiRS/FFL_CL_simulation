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

#ifndef AP_PSM_RESET_ORIGIN_REQUEST_PORT_H_
#define AP_PSM_RESET_ORIGIN_REQUEST_PORT_H_

#include "lsm_geoml/pose_pod.h"
#include "ap_psm/reset_origin_type.h"
#include "Platform_Types.h"


namespace ap_psm
{

  /// Reset request data in order to request the map reset from SI.
  struct ResetOriginRequestPort
  {
    ///@unit{nu}
    ///Relative to current origin, will be disregarded unless resetOrigin_nu equals RRT_RESET_CUSTOM. Contains target pose coordinates.
    ::lsm_geoml::Pose_POD transformation{};
    ///Type of the reset request.
    ResetOriginType resetOrigin_nu{};
    ///Will start from zero and increase by one on every reset request.
    uint8 resetCounter_nu{};
  };

} // namespace ap_psm

#endif // AP_PSM_RESET_ORIGIN_REQUEST_PORT_H_
