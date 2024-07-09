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

#ifndef AP_VEHSTATESIGPROVIDER_VEHICLE_OCCUPANCY_STATUS_PORT_INTERFACE_VERSION_H_
#define AP_VEHSTATESIGPROVIDER_VEHICLE_OCCUPANCY_STATUS_PORT_INTERFACE_VERSION_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace ap_vehstatesigprovider
{

  struct VehicleOccupancyStatusPort_InterfaceVersion
  {
    enum { VehicleOccupancyStatusPort_VERSION = 1U};
  };

  inline ::ap_vehstatesigprovider::VehicleOccupancyStatusPort_InterfaceVersion createVehicleOccupancyStatusPort_InterfaceVersion()
  {
    VehicleOccupancyStatusPort_InterfaceVersion m;
    (void)::eco::memset(&m, 0U, sizeof(VehicleOccupancyStatusPort_InterfaceVersion));
    return m;
  }

} // namespace ap_vehstatesigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_vehstatesigprovider::VehicleOccupancyStatusPort_InterfaceVersion create_default()
  {
      return ::ap_vehstatesigprovider::createVehicleOccupancyStatusPort_InterfaceVersion();
  }
}


#endif // AP_VEHSTATESIGPROVIDER_VEHICLE_OCCUPANCY_STATUS_PORT_INTERFACE_VERSION_H_
