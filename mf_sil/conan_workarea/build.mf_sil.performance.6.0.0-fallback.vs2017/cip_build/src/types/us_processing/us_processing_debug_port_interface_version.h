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

#ifndef US_PROCESSING_US_PROCESSING_DEBUG_PORT_INTERFACE_VERSION_H_
#define US_PROCESSING_US_PROCESSING_DEBUG_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_processing
{

  struct UsProcessingDebugPort_InterfaceVersion
  {
    enum { UsProcessingDebugPort_VERSION = 1U};
  };

  inline ::us_processing::UsProcessingDebugPort_InterfaceVersion createUsProcessingDebugPort_InterfaceVersion()
  {
    UsProcessingDebugPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(UsProcessingDebugPort_InterfaceVersion));
    return m;
  }

} // namespace us_processing

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_processing::UsProcessingDebugPort_InterfaceVersion create_default()
  {
      return ::us_processing::createUsProcessingDebugPort_InterfaceVersion();
  }
}


#endif // US_PROCESSING_US_PROCESSING_DEBUG_PORT_INTERFACE_VERSION_H_
