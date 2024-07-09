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

#ifndef PDCP_FC_PDCP_PARAMS_INTERFACE_VERSION_H_
#define PDCP_FC_PDCP_PARAMS_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace pdcp
{

  struct FC_PDCP_Params_InterfaceVersion
  {
    enum { FC_PDCP_Params_VERSION = 2U};
  };

  inline ::pdcp::FC_PDCP_Params_InterfaceVersion createFC_PDCP_Params_InterfaceVersion()
  {
    FC_PDCP_Params_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(FC_PDCP_Params_InterfaceVersion));
    return m;
  }

} // namespace pdcp

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::pdcp::FC_PDCP_Params_InterfaceVersion create_default()
  {
      return ::pdcp::createFC_PDCP_Params_InterfaceVersion();
  }
}


#endif // PDCP_FC_PDCP_PARAMS_INTERFACE_VERSION_H_