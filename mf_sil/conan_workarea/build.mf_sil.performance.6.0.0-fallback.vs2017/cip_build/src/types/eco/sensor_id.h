// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\typedef.h.template!

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

#ifndef ECO_SENSOR_ID_H_
#define ECO_SENSOR_ID_H_

#include "Platform_Types.h"

namespace eco
{

  /// ID provides the unique identification of the sensor
  typedef uint16 SensorID;

} // namespace eco

#endif // ECO_SENSOR_ID_H_
