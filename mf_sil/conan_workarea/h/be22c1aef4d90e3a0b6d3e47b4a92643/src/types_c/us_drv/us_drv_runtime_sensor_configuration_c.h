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

#ifndef US_DRV_US_DRV_RUNTIME_SENSOR_CONFIGURATION_C_H_
#define US_DRV_US_DRV_RUNTIME_SENSOR_CONFIGURATION_C_H_

#include "us_drv/us_drv_sensor_mode_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ///current/requested sensor operation mode and state
    US_DRV_UsDrvSensorMode state;
    ///delay values between burst pulses which form a stochastic code
    uint8 stochasticCodes_ms[8];
    ///@range{0,7}
    ///burst configuration for specific pulse
    uint8 firingSchemes[8];
    ///@range{0,1}
    ///selection of FIR coefficient set
    uint8 firCoefficientSet;
    ///@range{0,1}
    ///reception gain
    uint8 receiveGain;
    ///@range{0,1}
    uint8 advancedPathConfiguration;
    ///@range{0,3}
    ///Advanced Threshold Generator Path 1 configuration
    uint8 aatgConfiguration1;
    ///@range{0,3}
    ///Advanced Threshold Generator Path 2 configuration
    uint8 aatgConfiguration2;
    ///@range{0,1}
    ///select adaptive threshold generation
    uint8 adaptiveThresholdGeneration;
    ///major version of sensor firmware; read-only, can not be set
    uint8 swVersionMajor;
    ///minor version of sensor firmware; read-only, can not be set
    uint8 swVersionMinor;
    ///major version of sensor hardware; read-only, can not be set
    uint8 hwVersionMajor;
    ///minor version of sensor hardware; read-only, can not be set
    uint8 hwVersionMinor;
    ///sensor serial number; read-only, can not be set
    uint32 serialNumber;
} US_DRV_UsDrvRuntimeSensorConfiguration;

inline US_DRV_UsDrvRuntimeSensorConfiguration create_US_DRV_UsDrvRuntimeSensorConfiguration(void)
{
  US_DRV_UsDrvRuntimeSensorConfiguration m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // US_DRV_US_DRV_RUNTIME_SENSOR_CONFIGURATION_C_H_
