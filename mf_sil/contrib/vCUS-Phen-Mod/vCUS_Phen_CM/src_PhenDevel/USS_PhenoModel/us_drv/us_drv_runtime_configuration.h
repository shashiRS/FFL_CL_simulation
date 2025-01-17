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

#ifndef US_DRV_US_DRV_RUNTIME_CONFIGURATION_H_
#define US_DRV_US_DRV_RUNTIME_CONFIGURATION_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "Platform_Types.h"
#include "us_drv/us_drv_sensor_request.h"
#include "us_drv/us_drv_runtime_sensor_configuration.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvRuntimeConfiguration
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    uint8 sequence;
    UsDrvSensorRequest sensorRequests[18];
    uint8 responseSubState;
    uint8 numConfigurations;
    UsDrvRuntimeSensorConfiguration sensorConfigurations[18];
  };

  inline ::us_drv::UsDrvRuntimeConfiguration createUsDrvRuntimeConfiguration()
  {
    UsDrvRuntimeConfiguration m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvRuntimeConfiguration));
    m.sSigHeader = ::eco::createSignalHeader();
    {
      const uint64 arraysize = (sizeof(m.sensorRequests) / sizeof(m.sensorRequests[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.sensorRequests[i] = createUsDrvSensorRequest();
      }
    }
    {
      const uint64 arraysize = (sizeof(m.sensorConfigurations) / sizeof(m.sensorConfigurations[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.sensorConfigurations[i] = createUsDrvRuntimeSensorConfiguration();
      }
    }
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvRuntimeConfiguration create_default()
  {
      return ::us_drv::createUsDrvRuntimeConfiguration();
  }
}


#endif // US_DRV_US_DRV_RUNTIME_CONFIGURATION_H_
