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

#ifndef AP_VEHSTATESIGPROVIDER_VEHICLE_OCCUPANCY_STATUS_PORT_H_
#define AP_VEHSTATESIGPROVIDER_VEHICLE_OCCUPANCY_STATUS_PORT_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "ap_vehstatesigprovider/seat_occupancy_status.h"
#include "ap_vehstatesigprovider/belt_buckle_status.h"
#include "eco/memset.h"


namespace ap_vehstatesigprovider
{

  /// Signals from Airbag ECU
  struct VehicleOccupancyStatusPort
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    ///@unit{nu}
    ///Status of the seat occupancy for each seat
    SeatOccupancyStatus seatOccupancyStatus;
    ///@unit{nu}
    ///Status of the belt buckle for each seat
    BeltBuckleStatus beltBuckleStatus;
  };

  inline ::ap_vehstatesigprovider::VehicleOccupancyStatusPort createVehicleOccupancyStatusPort()
  {
    VehicleOccupancyStatusPort m;
    (void)::eco::memset(&m, 0U, sizeof(VehicleOccupancyStatusPort));
    m.sSigHeader = ::eco::createSignalHeader();
    m.seatOccupancyStatus = createSeatOccupancyStatus();
    m.beltBuckleStatus = createBeltBuckleStatus();
    return m;
  }

} // namespace ap_vehstatesigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_vehstatesigprovider::VehicleOccupancyStatusPort create_default()
  {
      return ::ap_vehstatesigprovider::createVehicleOccupancyStatusPort();
  }
}


#endif // AP_VEHSTATESIGPROVIDER_VEHICLE_OCCUPANCY_STATUS_PORT_H_
