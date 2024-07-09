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

#ifndef US_DRV_US_DRV_DEBUG_PORT_INTERFACE_VERSION_H_
#define US_DRV_US_DRV_DEBUG_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvDebugPort_InterfaceVersion
  {
    enum { UsDrvDebugPort_VERSION = 1U};
  };

  inline ::us_drv::UsDrvDebugPort_InterfaceVersion createUsDrvDebugPort_InterfaceVersion()
  {
    UsDrvDebugPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvDebugPort_InterfaceVersion));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvDebugPort_InterfaceVersion create_default()
  {
      return ::us_drv::createUsDrvDebugPort_InterfaceVersion();
  }
}


#endif // US_DRV_US_DRV_DEBUG_PORT_INTERFACE_VERSION_H_