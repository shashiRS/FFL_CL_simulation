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

#ifndef US_PROCESSING_US_PROCESSING_DATA_INTEGRITY_C_H_
#define US_PROCESSING_US_PROCESSING_DATA_INTEGRITY_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "us_processing/us_processing_sensor_running_mode_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,4}
    ///@unit{nu}
    ///Sensor running mode in current cycle
    US_PROCESSING_UsProcessingSensorRunningMode sensorRunningMode[12];
    ///@range{0.0F,7.0F}
    ///@unit{m}
    ///Filtered sensor distance in current cycle
    float32 filtDist_m[12];
    ///@range{0,500}
    ///@unit{nu}
    ///Sensor raw echo count in current cycle
    uint16 rawEchoCount[12];
    ///@range{-100.0F,100.0F}
    ///@unit{mps}
    ///Input velocity to UDP in current cycle
    float32 inputVelocity;
    ///@range{-40.0F,95.0F}
    ///@unit{°C}
    ///Input temperature to UDP in current cycle
    float32 inputTemperature;
    ///@range{0,1}
    ///@unit{nu}
    ///Input data to UDP in SW Devkit is ok
    boolean isInputCommOk;
} US_PROCESSING_UsProcessingDataIntegrity;

inline US_PROCESSING_UsProcessingDataIntegrity create_US_PROCESSING_UsProcessingDataIntegrity(void)
{
  US_PROCESSING_UsProcessingDataIntegrity m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // US_PROCESSING_US_PROCESSING_DATA_INTEGRITY_C_H_
