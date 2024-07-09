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

#ifndef PDCP_PDCPDRIVING_TUBE_PORT_INTERFACE_VERSION_H_
#define PDCP_PDCPDRIVING_TUBE_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace pdcp
{

  struct PDCPDrivingTubePort_InterfaceVersion
  {
    enum { PDCPDrivingTubePort_VERSION = 1U};
  };

  inline ::pdcp::PDCPDrivingTubePort_InterfaceVersion createPDCPDrivingTubePort_InterfaceVersion()
  {
    PDCPDrivingTubePort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(PDCPDrivingTubePort_InterfaceVersion));
    return m;
  }

} // namespace pdcp

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::pdcp::PDCPDrivingTubePort_InterfaceVersion create_default()
  {
      return ::pdcp::createPDCPDrivingTubePort_InterfaceVersion();
  }
}


#endif // PDCP_PDCPDRIVING_TUBE_PORT_INTERFACE_VERSION_H_