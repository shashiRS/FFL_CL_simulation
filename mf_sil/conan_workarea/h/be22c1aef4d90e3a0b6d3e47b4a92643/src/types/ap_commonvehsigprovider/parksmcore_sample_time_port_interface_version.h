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

#ifndef AP_COMMONVEHSIGPROVIDER_PARKSMCORE_SAMPLE_TIME_PORT_INTERFACE_VERSION_H_
#define AP_COMMONVEHSIGPROVIDER_PARKSMCORE_SAMPLE_TIME_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_commonvehsigprovider
{

  struct PARKSMCoreSampleTimePort_InterfaceVersion
  {
    enum { PARKSMCoreSampleTimePort_VERSION = 1U};
  };

  inline ::ap_commonvehsigprovider::PARKSMCoreSampleTimePort_InterfaceVersion createPARKSMCoreSampleTimePort_InterfaceVersion()
  {
    PARKSMCoreSampleTimePort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(PARKSMCoreSampleTimePort_InterfaceVersion));
    return m;
  }

} // namespace ap_commonvehsigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_commonvehsigprovider::PARKSMCoreSampleTimePort_InterfaceVersion create_default()
  {
      return ::ap_commonvehsigprovider::createPARKSMCoreSampleTimePort_InterfaceVersion();
  }
}


#endif // AP_COMMONVEHSIGPROVIDER_PARKSMCORE_SAMPLE_TIME_PORT_INTERFACE_VERSION_H_
