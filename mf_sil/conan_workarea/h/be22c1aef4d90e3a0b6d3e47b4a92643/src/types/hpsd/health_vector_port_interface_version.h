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

#ifndef HPSD_HEALTH_VECTOR_PORT_INTERFACE_VERSION_H_
#define HPSD_HEALTH_VECTOR_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace hpsd
{

  struct HealthVectorPort_InterfaceVersion
  {
    enum { HealthVectorPort_VERSION = 1U};
  };

  inline ::hpsd::HealthVectorPort_InterfaceVersion createHealthVectorPort_InterfaceVersion()
  {
    HealthVectorPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(HealthVectorPort_InterfaceVersion));
    return m;
  }

} // namespace hpsd

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::hpsd::HealthVectorPort_InterfaceVersion create_default()
  {
      return ::hpsd::createHealthVectorPort_InterfaceVersion();
  }
}


#endif // HPSD_HEALTH_VECTOR_PORT_INTERFACE_VERSION_H_
