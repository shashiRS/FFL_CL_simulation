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

#ifndef MF_LSCA_LSCA_STATUS_PORT_INTERFACE_VERSION_H_
#define MF_LSCA_LSCA_STATUS_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_lsca
{

  struct LscaStatusPort_InterfaceVersion
  {
    enum { LscaStatusPort_VERSION = 1U};
  };

  inline ::mf_lsca::LscaStatusPort_InterfaceVersion createLscaStatusPort_InterfaceVersion()
  {
    LscaStatusPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(LscaStatusPort_InterfaceVersion));
    return m;
  }

} // namespace mf_lsca

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_lsca::LscaStatusPort_InterfaceVersion create_default()
  {
      return ::mf_lsca::createLscaStatusPort_InterfaceVersion();
  }
}


#endif // MF_LSCA_LSCA_STATUS_PORT_INTERFACE_VERSION_H_