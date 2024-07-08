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

#ifndef ECO_SENSOR_MOUNT_POSITION_H_
#define ECO_SENSOR_MOUNT_POSITION_H_

#include "eco/signal_header.h"
#include "eco/sensor_id.h"
#include "eco/sensor_mounting_position_detailed.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace eco
{

  /// Sensor mount position and sensor ID
  struct SensorMountPosition
  {

  ///General information about the signal
    SignalHeader sigHeader;

  ///Sensor ID of the related sensor
    SensorID sensorId;

  ///Mounting position details of the sensor relative to the environment model
  ///coordinate system
    SensorMountingPositionDetailed sensorMountPos;

  ///Specifies if sensor mount position is available or not
    boolean isSensorMountPosAvailable;
  };

  inline ::eco::SensorMountPosition createSensorMountPosition()
  {
    SensorMountPosition m;
    (void)::eco::memset(&m, 0U, sizeof(SensorMountPosition));
    m.sigHeader = createSignalHeader();
    m.sensorMountPos = createSensorMountingPositionDetailed();
    return m;
  }

} // namespace eco

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::eco::SensorMountPosition create_default()
  {
      return ::eco::createSensorMountPosition();
  }
}


#endif // ECO_SENSOR_MOUNT_POSITION_H_
