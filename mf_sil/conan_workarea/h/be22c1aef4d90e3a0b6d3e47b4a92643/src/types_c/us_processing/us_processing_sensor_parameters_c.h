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

#ifndef US_PROCESSING_US_PROCESSING_SENSOR_PARAMETERS_C_H_
#define US_PROCESSING_US_PROCESSING_SENSOR_PARAMETERS_C_H_

#include "Platform_Types.h"
#include "us_processing/us_processing_sensor_parameter_c.h"
#include "eco/memset_c.h"

typedef struct
{
    uint32 sensorParameterCount;
    US_PROCESSING_UsProcessingSensorParameter sensorParameter[12];
} US_PROCESSING_UsProcessingSensorParameters;

inline US_PROCESSING_UsProcessingSensorParameters create_US_PROCESSING_UsProcessingSensorParameters(void)
{
  US_PROCESSING_UsProcessingSensorParameters m;
  (void) ECO_memset (&m, 0, sizeof(m));
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.sensorParameter) / sizeof(m.sensorParameter[0])); ++i)
    {
      m.sensorParameter[i] = create_US_PROCESSING_UsProcessingSensorParameter();
    }
  }
  return m;
}

#endif // US_PROCESSING_US_PROCESSING_SENSOR_PARAMETERS_C_H_
