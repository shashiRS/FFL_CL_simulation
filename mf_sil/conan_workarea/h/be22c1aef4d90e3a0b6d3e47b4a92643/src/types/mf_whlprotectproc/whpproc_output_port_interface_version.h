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

#ifndef MF_WHLPROTECTPROC_WHPPROC_OUTPUT_PORT_INTERFACE_VERSION_H_
#define MF_WHLPROTECTPROC_WHPPROC_OUTPUT_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_whlprotectproc
{

  struct WHPProcOutputPort_InterfaceVersion
  {
    enum { WHPProcOutputPort_VERSION = 1U};
  };

  inline ::mf_whlprotectproc::WHPProcOutputPort_InterfaceVersion createWHPProcOutputPort_InterfaceVersion()
  {
    WHPProcOutputPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(WHPProcOutputPort_InterfaceVersion));
    return m;
  }

} // namespace mf_whlprotectproc

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_whlprotectproc::WHPProcOutputPort_InterfaceVersion create_default()
  {
      return ::mf_whlprotectproc::createWHPProcOutputPort_InterfaceVersion();
  }
}


#endif // MF_WHLPROTECTPROC_WHPPROC_OUTPUT_PORT_INTERFACE_VERSION_H_
