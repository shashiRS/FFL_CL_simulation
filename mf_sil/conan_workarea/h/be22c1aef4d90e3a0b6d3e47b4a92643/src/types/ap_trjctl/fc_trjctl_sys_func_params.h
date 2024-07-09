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

#ifndef AP_TRJCTL_FC_TRJCTL_SYS_FUNC_PARAMS_H_
#define AP_TRJCTL_FC_TRJCTL_SYS_FUNC_PARAMS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_trjctl
{

  struct FC_TRJCTL_Sys_Func_Params
  {
    ///Maximum lateral acceleration of the ego-vehicle
    float32 AP_G_MAX_AVG_LAT_ACCEL_MPS2;
    ///Maximum yaw rate of the ego-vehicle
    float32 AP_G_MAX_AVG_YAW_RATE_RADPS;
  };

  inline ::ap_trjctl::FC_TRJCTL_Sys_Func_Params createFC_TRJCTL_Sys_Func_Params()
  {
    FC_TRJCTL_Sys_Func_Params m;
    (void)::eco::memset(&m, 0U, sizeof(FC_TRJCTL_Sys_Func_Params));
    return m;
  }

} // namespace ap_trjctl

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_trjctl::FC_TRJCTL_Sys_Func_Params create_default()
  {
      return ::ap_trjctl::createFC_TRJCTL_Sys_Func_Params();
  }
}


#endif // AP_TRJCTL_FC_TRJCTL_SYS_FUNC_PARAMS_H_