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

#ifndef ECO_SENSOR_MOUNT_POSITION_H_
#define ECO_SENSOR_MOUNT_POSITION_H_

#include "eco/signal_header.h"
#include "eco/sensor_id.h"
#include "eco/sensor_mounting_position_detailed.h"
#include "Platform_Types.h"


namespace eco
{

  /// Sensor mount position and sensor ID
  struct SensorMountPosition
  {
    ///General information about the signal
    SignalHeader sigHeader{};
    ///Sensor ID of the related sensor
    SensorID sensorId{};
    ///Mounting position details of the sensor relative to the environment model
    ///coordinate system
    SensorMountingPositionDetailed sensorMountPos{};
    ///Specifies if sensor mount position is available or not
    boolean isSensorMountPosAvailable{};
  };

} // namespace eco

#endif // ECO_SENSOR_MOUNT_POSITION_H_
