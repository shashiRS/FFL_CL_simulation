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

#ifndef AP_TRJCTL_FC_TRJCTL_VEHICLE_PARAMS_H_
#define AP_TRJCTL_FC_TRJCTL_VEHICLE_PARAMS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_trjctl
{

  struct FC_TRJCTL_Vehicle_Params
  {
    ///Maximum steering angle (at wheels).
    float32 AP_V_MAX_STEER_ANG_RAD;
  };

  inline ::ap_trjctl::FC_TRJCTL_Vehicle_Params createFC_TRJCTL_Vehicle_Params()
  {
    FC_TRJCTL_Vehicle_Params m;
    (void)::eco::memset(&m, 0U, sizeof(FC_TRJCTL_Vehicle_Params));
    return m;
  }

} // namespace ap_trjctl

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_trjctl::FC_TRJCTL_Vehicle_Params create_default()
  {
      return ::ap_trjctl::createFC_TRJCTL_Vehicle_Params();
  }
}


#endif // AP_TRJCTL_FC_TRJCTL_VEHICLE_PARAMS_H_