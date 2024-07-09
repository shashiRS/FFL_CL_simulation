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

#ifndef AP_VEHSTATESIGPROVIDER_CHARGING_STATUS_H_
#define AP_VEHSTATESIGPROVIDER_CHARGING_STATUS_H_

#include "Platform_Types.h"
#include "ap_vehstatesigprovider/charging_connector_status.h"
#include "eco/memset.h"


namespace ap_vehstatesigprovider
{

  /// Status of the EV charging system.
  struct ChargingStatus
  {
    boolean ev_charging_is_installed_nu;
    ///Operating condition of the EV battery charging port connector.
    ChargingConnectorStatus chargingConnectorStatus_nu;
  };

  inline ::ap_vehstatesigprovider::ChargingStatus createChargingStatus()
  {
    ChargingStatus m;
    (void)::eco::memset(&m, 0U, sizeof(ChargingStatus));
    return m;
  }

} // namespace ap_vehstatesigprovider

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::ap_vehstatesigprovider::ChargingStatus create_default()
  {
      return ::ap_vehstatesigprovider::createChargingStatus();
  }
}


#endif // AP_VEHSTATESIGPROVIDER_CHARGING_STATUS_H_
