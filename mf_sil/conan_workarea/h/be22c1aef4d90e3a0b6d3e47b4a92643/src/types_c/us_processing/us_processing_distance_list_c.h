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

#ifndef US_PROCESSING_US_PROCESSING_DISTANCE_LIST_C_H_
#define US_PROCESSING_US_PROCESSING_DISTANCE_LIST_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,US_PROCESSING_MAX_NUM_SENSORS}
    ///@unit{nu}
    ///used data buffers to optimize processing
    uint8 numberOfSensors;
    ///@range{0.0F,14.0F}
    ///@unit{m}
    ///Sensor direct signal way (Traveled distance) measured
    float32 distdir_m[12];
    ///@range{-3.4x10^38,3.4x10^38}
    ///@unit{nu}
    ///Quality of Sensor direct signal way (Traveled distance) measured
    float32 distDirQuality[12];
    ///@range{0.0F,14.0F}
    ///@unit{m}
    ///Sensor cross signal way (Traveled distance) measured
    float32 distCross_m[12];
    ///@range{-3.4x10^38,3.4x10^38}
    ///@unit{nu}
    ///Quality of Sensor cross signal way (Traveled distance) measured
    float32 distCrossQuality[12];
} US_PROCESSING_UsProcessingDistanceList;

inline US_PROCESSING_UsProcessingDistanceList create_US_PROCESSING_UsProcessingDistanceList(void)
{
  US_PROCESSING_UsProcessingDistanceList m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  return m;
}

#endif // US_PROCESSING_US_PROCESSING_DISTANCE_LIST_C_H_
