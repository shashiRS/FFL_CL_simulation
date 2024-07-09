//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef US_DRV_US_DRV_RUNTIME_CONFIGURATION_C_H_
#define US_DRV_US_DRV_RUNTIME_CONFIGURATION_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "us_drv/us_drv_sensor_request_c.h"
#include "us_drv/us_drv_runtime_sensor_configuration_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,255}
    ///@unit{nu}
    ///Request sequence number
    uint8 sequence;
    ///Request for each sensor
    US_DRV_UsDrvSensorRequest sensorRequests[18];
    ///@range{0,255}
    ///@unit{nu}
    ///Sub state of response
    uint8 responseSubState;
    ///@range{0,US_DRV_MAX_NUM_SENSORS}
    ///@unit{nu}
    ///Number of configurations
    uint8 numConfigurations;
    ///Array of sensor configurations
    US_DRV_UsDrvRuntimeSensorConfiguration sensorConfigurations[18];
} US_DRV_UsDrvRuntimeConfiguration;

inline US_DRV_UsDrvRuntimeConfiguration create_US_DRV_UsDrvRuntimeConfiguration(void)
{
  US_DRV_UsDrvRuntimeConfiguration m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.sensorRequests) / sizeof(m.sensorRequests[0])); ++i)
    {
      m.sensorRequests[i] = create_US_DRV_UsDrvSensorRequest();
    }
  }
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.sensorConfigurations) / sizeof(m.sensorConfigurations[0])); ++i)
    {
      m.sensorConfigurations[i] = create_US_DRV_UsDrvRuntimeSensorConfiguration();
    }
  }
  return m;
}

#endif // US_DRV_US_DRV_RUNTIME_CONFIGURATION_C_H_
