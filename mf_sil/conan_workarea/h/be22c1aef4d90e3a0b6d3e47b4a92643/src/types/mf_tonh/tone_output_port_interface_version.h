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

#ifndef MF_TONH_TONE_OUTPUT_PORT_INTERFACE_VERSION_H_
#define MF_TONH_TONE_OUTPUT_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace mf_tonh
{

  struct ToneOutputPort_InterfaceVersion
  {
    enum { ToneOutputPort_VERSION = 1U};
  };

  inline ::mf_tonh::ToneOutputPort_InterfaceVersion createToneOutputPort_InterfaceVersion()
  {
    ToneOutputPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(ToneOutputPort_InterfaceVersion));
    return m;
  }

} // namespace mf_tonh

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::mf_tonh::ToneOutputPort_InterfaceVersion create_default()
  {
      return ::mf_tonh::createToneOutputPort_InterfaceVersion();
  }
}


#endif // MF_TONH_TONE_OUTPUT_PORT_INTERFACE_VERSION_H_
