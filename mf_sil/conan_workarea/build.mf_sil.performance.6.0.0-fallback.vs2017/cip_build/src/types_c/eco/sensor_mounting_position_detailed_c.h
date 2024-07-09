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

#ifndef ECO_SENSOR_MOUNTING_POSITION_DETAILED_C_H_
#define ECO_SENSOR_MOUNTING_POSITION_DETAILED_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// Mounting position details of the sensor relative to the environment model
/// coordinate system
typedef struct
{
    ///@unit{rad}
    ///Rolling angle of the sensor in environment model coordinate system
    float32 roll;
    ///@unit{rad}
    ///Pitch angle of the sensor in environment model coordinate system
    float32 pitch;
    ///@unit{rad}
    ///Yaw angle of the sensorin environment model coordinate system
    float32 yaw;
    ///@unit{meters}
    ///X-coordinate position of the sensor in environment model coordinate system
    float32 x;
    ///@unit{meters}
    ///Y-coordinate position of the sensor in environment model coordinate system
    float32 y;
    ///@unit{meters}
    ///Z-coordinate position of the sensor in environment model coordinate system
    float32 z;
} ECO_SensorMountingPositionDetailed;

inline ECO_SensorMountingPositionDetailed create_ECO_SensorMountingPositionDetailed(void)
{
  ECO_SensorMountingPositionDetailed m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // ECO_SENSOR_MOUNTING_POSITION_DETAILED_C_H_