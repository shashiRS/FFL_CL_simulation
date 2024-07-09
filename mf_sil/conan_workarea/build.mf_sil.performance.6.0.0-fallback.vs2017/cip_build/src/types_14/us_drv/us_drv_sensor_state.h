// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

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

#ifndef US_DRV_US_DRV_SENSOR_STATE_H_
#define US_DRV_US_DRV_SENSOR_STATE_H_

#include "Platform_Types.h"

namespace us_drv
{
  enum class UsDrvSensorState : uint8
  {
      ///Sensor is off.
      US_SENSOR_STATE_OFF = 0U,
      ///Sensor is ready but not firing.
      US_SENSOR_STATE_STANDBY = 1U,
      ///Sensor is in process of initialization
      US_SENSOR_STATE_INITIALISING = 2U,
      ///Sensor is in normal/active operation mode.
      US_SENSOR_STATE_RUNNING_NORMAL_MODE = 3U,
      ///Sensor is in diagnostic RX/TX mode.
      US_SENSOR_STATE_RUNNING_DIAGNOSTIC_MODE = 4U,
      ///Sensor is in error state.
      US_SENSOR_STATE_ERROR = 128U,
  };
} // namespace us_drv
#endif // US_DRV_US_DRV_SENSOR_STATE_H_