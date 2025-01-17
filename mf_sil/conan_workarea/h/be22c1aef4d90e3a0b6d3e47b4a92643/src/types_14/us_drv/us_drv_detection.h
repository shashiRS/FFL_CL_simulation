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

#ifndef US_DRV_US_DRV_DETECTION_H_
#define US_DRV_US_DRV_DETECTION_H_

#include "Platform_Types.h"
#include "us_drv/us_drv_detection_type.h"


namespace us_drv
{

  struct UsDrvDetection
  {
    ///@range{1,US_DRV_MAX_NUM_SENSORS}
    ///logical sensor id of receiving sensor
    uint8 sensorId{};
    UsDrvDetectionType detectionType{};
    ///@range{0,65535}
    ///@unit{nu}
    ///measured amplitude of detection
    uint16 amplitude{};
    ///@range{-128,127}
    ///@unit{nu}
    ///phase derivative value
    sint8 phaseDerivative{};
    ///@unit{ms}
    ///@range{0,255}
    ///PDCM sync pulse count when this measurement was transmitted to ECU
    uint8 syncCnt_ms{};
    ///@range{0,65535}
    ///@unit{us}
    ///sensor internal timer timestamp of detection
    uint16 sensorTimestamp_us{};
    ///@range{-2147483648,2147483647}
    ///@unit{us}
    ///relative timestamp when measurement was received by ECU. this is a negative offset to UsDrvDetectionList.sSigHeader.uiTimestamp
    sint32 relEcuTimestamp_us{};
  };

} // namespace us_drv

#endif // US_DRV_US_DRV_DETECTION_H_
