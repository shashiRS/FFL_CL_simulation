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

#ifndef US_DRV_US_DRV_DETECTION_H_
#define US_DRV_US_DRV_DETECTION_H_

#include "Platform_Types.h"
#include "us_drv/us_drv_detection_type.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvDetection
  {
    uint8 sensorId;
    UsDrvDetectionType detectionType;
    uint16 amplitude;
    sint8 phaseDerivative;
    uint8 syncCnt_ms;
    uint16 sensorTimestamp_us;
    sint32 relEcuTimestamp_us;
  };

  inline ::us_drv::UsDrvDetection createUsDrvDetection()
  {
    UsDrvDetection m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvDetection));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvDetection create_default()
  {
      return ::us_drv::createUsDrvDetection();
  }
}


#endif // US_DRV_US_DRV_DETECTION_H_
