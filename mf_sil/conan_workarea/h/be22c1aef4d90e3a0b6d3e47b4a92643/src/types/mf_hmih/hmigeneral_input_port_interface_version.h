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

#ifndef MF_HMIH_HMIGENERAL_INPUT_PORT_INTERFACE_VERSION_H_
#define MF_HMIH_HMIGENERAL_INPUT_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_hmih
{

  struct HMIGeneralInputPort_InterfaceVersion
  {
    enum { HMIGeneralInputPort_VERSION = 8U};
  };

  inline ::mf_hmih::HMIGeneralInputPort_InterfaceVersion createHMIGeneralInputPort_InterfaceVersion()
  {
    HMIGeneralInputPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(HMIGeneralInputPort_InterfaceVersion));
    return m;
  }

} // namespace mf_hmih

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_hmih::HMIGeneralInputPort_InterfaceVersion create_default()
  {
      return ::mf_hmih::createHMIGeneralInputPort_InterfaceVersion();
  }
}


#endif // MF_HMIH_HMIGENERAL_INPUT_PORT_INTERFACE_VERSION_H_
