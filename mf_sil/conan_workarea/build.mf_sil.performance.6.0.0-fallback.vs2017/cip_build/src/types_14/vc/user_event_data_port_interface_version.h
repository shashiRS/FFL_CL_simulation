// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef VC_USER_EVENT_DATA_PORT_INTERFACE_VERSION_H_
#define VC_USER_EVENT_DATA_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"


namespace vc
{

  struct UserEventDataPort_InterfaceVersion
  {
    static constexpr uint32 UserEventDataPort_VERSION = 1U;
  };

} // namespace vc

#endif // VC_USER_EVENT_DATA_PORT_INTERFACE_VERSION_H_
