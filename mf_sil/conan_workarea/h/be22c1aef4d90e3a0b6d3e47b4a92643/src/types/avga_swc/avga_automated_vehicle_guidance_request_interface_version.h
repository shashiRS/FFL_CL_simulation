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

#ifndef AVGA_SWC_AVGA_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_INTERFACE_VERSION_H_
#define AVGA_SWC_AVGA_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace avga_swc
{

  struct AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion
  {
    enum { AVGA_AutomatedVehicleGuidanceRequest_VERSION = 1U};
  };

  inline ::avga_swc::AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion createAVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion()
  {
    AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion));
    return m;
  }

} // namespace avga_swc

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::avga_swc::AVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion create_default()
  {
      return ::avga_swc::createAVGA_AutomatedVehicleGuidanceRequest_InterfaceVersion();
  }
}


#endif // AVGA_SWC_AVGA_AUTOMATED_VEHICLE_GUIDANCE_REQUEST_INTERFACE_VERSION_H_
