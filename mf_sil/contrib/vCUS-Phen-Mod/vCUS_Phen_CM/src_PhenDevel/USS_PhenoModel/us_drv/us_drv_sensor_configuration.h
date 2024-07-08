// Attention, this file is generated by Cobolt from template: C:\_repos\mf_sil\dbg\eco\eco.generic\codegen\templates\types\struct.h.template!

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

#ifndef US_DRV_US_DRV_SENSOR_CONFIGURATION_H_
#define US_DRV_US_DRV_SENSOR_CONFIGURATION_H_

#include "Platform_Types.h"
#include "us_drv/us_drv_runtime_sensor_configuration.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvSensorConfiguration
  {
    uint8 bus;
    uint8 physicalAddress;
    UsDrvRuntimeSensorConfiguration initialConfiguration;
  };

  inline ::us_drv::UsDrvSensorConfiguration createUsDrvSensorConfiguration()
  {
    UsDrvSensorConfiguration m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvSensorConfiguration));
    m.initialConfiguration = createUsDrvRuntimeSensorConfiguration();
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvSensorConfiguration create_default()
  {
      return ::us_drv::createUsDrvSensorConfiguration();
  }
}


#endif // US_DRV_US_DRV_SENSOR_CONFIGURATION_H_
