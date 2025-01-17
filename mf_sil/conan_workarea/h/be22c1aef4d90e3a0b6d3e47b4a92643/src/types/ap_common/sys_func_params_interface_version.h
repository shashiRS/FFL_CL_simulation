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

#ifndef AP_COMMON_SYS_FUNC_PARAMS_INTERFACE_VERSION_H_
#define AP_COMMON_SYS_FUNC_PARAMS_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_common
{

  struct Sys_Func_Params_InterfaceVersion
  {
    enum { Sys_Func_Params_VERSION = 1U};
  };

  inline ::ap_common::Sys_Func_Params_InterfaceVersion createSys_Func_Params_InterfaceVersion()
  {
    Sys_Func_Params_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(Sys_Func_Params_InterfaceVersion));
    return m;
  }

} // namespace ap_common

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_common::Sys_Func_Params_InterfaceVersion create_default()
  {
      return ::ap_common::createSys_Func_Params_InterfaceVersion();
  }
}


#endif // AP_COMMON_SYS_FUNC_PARAMS_INTERFACE_VERSION_H_
