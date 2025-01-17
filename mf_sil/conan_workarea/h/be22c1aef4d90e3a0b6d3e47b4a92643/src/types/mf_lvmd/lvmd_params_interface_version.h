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

#ifndef MF_LVMD_LVMD_PARAMS_INTERFACE_VERSION_H_
#define MF_LVMD_LVMD_PARAMS_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_lvmd
{

  struct LvmdParams_InterfaceVersion
  {
    enum { LvmdParams_VERSION = 1U};
  };

  inline ::mf_lvmd::LvmdParams_InterfaceVersion createLvmdParams_InterfaceVersion()
  {
    LvmdParams_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(LvmdParams_InterfaceVersion));
    return m;
  }

} // namespace mf_lvmd

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lvmd::LvmdParams_InterfaceVersion create_default()
  {
      return ::mf_lvmd::createLvmdParams_InterfaceVersion();
  }
}


#endif // MF_LVMD_LVMD_PARAMS_INTERFACE_VERSION_H_
