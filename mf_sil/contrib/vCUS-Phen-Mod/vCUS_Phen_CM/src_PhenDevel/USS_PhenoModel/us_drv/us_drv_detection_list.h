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

#ifndef US_DRV_US_DRV_DETECTION_LIST_H_
#define US_DRV_US_DRV_DETECTION_LIST_H_

#include "eco/algo_interface_version_number.h"
#include "eco/signal_header.h"
#include "us_drv/us_drv_sensor_state.h"
#include "Platform_Types.h"
#include "us_drv/us_drv_detection.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvDetectionList
  {
    ::eco::AlgoInterfaceVersionNumber uiVersionNumber;
    ::eco::SignalHeader sSigHeader;
    UsDrvSensorState sensorState[18];
    uint16 numDetections;
    UsDrvDetection detections[1800];
  };

  inline ::us_drv::UsDrvDetectionList createUsDrvDetectionList()
  {
    UsDrvDetectionList m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvDetectionList));
    m.sSigHeader = ::eco::createSignalHeader();
    {
      const uint64 arraysize = (sizeof(m.detections) / sizeof(m.detections[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.detections[i] = createUsDrvDetection();
      }
    }
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvDetectionList create_default()
  {
      return ::us_drv::createUsDrvDetectionList();
  }
}


#endif // US_DRV_US_DRV_DETECTION_LIST_H_