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

#ifndef US_PROCESSING_US_PROCESSING_POINT_LIST_C_H_
#define US_PROCESSING_US_PROCESSING_POINT_LIST_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "Platform_Types.h"
#include "us_processing/us_processing_point_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    ///@range{0,US_PROCESSING_MAX_NUM_POINTS}
    ///@unit{nu}
    ///used data buffers to optimize processing
    uint32 numberOfPoints;
    ///Array of Points
    US_PROCESSING_UsProcessingPoint points[255];
} US_PROCESSING_UsProcessingPointList;

inline US_PROCESSING_UsProcessingPointList create_US_PROCESSING_UsProcessingPointList(void)
{
  US_PROCESSING_UsProcessingPointList m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.points) / sizeof(m.points[0])); ++i)
    {
      m.points[i] = create_US_PROCESSING_UsProcessingPoint();
    }
  }
  return m;
}

#endif // US_PROCESSING_US_PROCESSING_POINT_LIST_C_H_