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

#ifndef AP_LODMC_LO_DMCSTATUS_PORT_INTERFACE_VERSION_H_
#define AP_LODMC_LO_DMCSTATUS_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"


namespace ap_lodmc
{

  struct LoDMCStatusPort_InterfaceVersion
  {
    static constexpr uint32 LoDMCStatusPort_VERSION = 2U;
  };

} // namespace ap_lodmc

#endif // AP_LODMC_LO_DMCSTATUS_PORT_INTERFACE_VERSION_H_