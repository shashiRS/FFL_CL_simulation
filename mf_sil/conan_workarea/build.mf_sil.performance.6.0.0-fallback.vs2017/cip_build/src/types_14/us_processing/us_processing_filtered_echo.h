// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef US_PROCESSING_US_PROCESSING_FILTERED_ECHO_H_
#define US_PROCESSING_US_PROCESSING_FILTERED_ECHO_H_

#include "Platform_Types.h"


namespace us_processing
{

  struct UsProcessingFilteredEcho
  {
    ///@range{1,US_PROCESSING_MAX_NUM_SENSORS}
    ///@unit{nu}
    ///Sensor Id of receiving sensor
    uint8 rxSensorId{};
    ///@range{1,US_PROCESSING_MAX_NUM_SENSORS}
    ///@unit{nu}
    ///Sensor Id of transmitting sensor
    uint8 txSensorId{};
    ///@range{1,65535}
    ///@unit{nu}
    ///Track Id of correlation track
    uint16 trackId{};
    ///@range{-2147483648,2147483647}
    ///@unit{us}
    ///Relative timestamp when measurement was received by ECU. This is a negative offset to UsProcessingFilteredEcho.sSigHeader.uiTimestamp
    sint32 relEcuTimestamp_us{};
    ///@range{0,65535}
    ///@unit{us}
    ///Echo time of flight
    uint16 tof_us{};
    uint16 amplitude{};
    ///@range{-128,127}
    ///@unit{nu}
    ///Phase derivative value
    sint8 phaseDerivative{};
    ///@range{0,15}
    ///@unit{nu}
    ///Frequency domain coding cofidence level
    uint8 codingConfidence{};
    ///@range{0,15}
    ///@unit{nu}
    ///Time domain tracking confidence level
    uint8 timeConfidence{};
  };

} // namespace us_processing

#endif // US_PROCESSING_US_PROCESSING_FILTERED_ECHO_H_
