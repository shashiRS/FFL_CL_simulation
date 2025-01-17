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

#ifndef US_DRV_US_DRV_CONFIGURATION_C_H_
#define US_DRV_US_DRV_CONFIGURATION_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "us_drv/us_drv_sensor_configuration_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    US_DRV_UsDrvSensorConfiguration sensorConfiguration[18];
} US_DRV_UsDrvConfiguration;

inline US_DRV_UsDrvConfiguration create_US_DRV_UsDrvConfiguration(void)
{
  US_DRV_UsDrvConfiguration m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.sensorConfiguration) / sizeof(m.sensorConfiguration[0])); ++i)
    {
      m.sensorConfiguration[i] = create_US_DRV_UsDrvSensorConfiguration();
    }
  }
  return m;
}

#endif // US_DRV_US_DRV_CONFIGURATION_C_H_
