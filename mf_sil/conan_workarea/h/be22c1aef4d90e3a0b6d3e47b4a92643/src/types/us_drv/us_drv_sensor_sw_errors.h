// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

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

#ifndef US_DRV_US_DRV_SENSOR_SW_ERRORS_H_
#define US_DRV_US_DRV_SENSOR_SW_ERRORS_H_

#include "us_drv/us_drv_error_status.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvSensorSwErrors
  {
    UsDrvErrorStatus sensorInitTimeout;
    uint8 sensorInitTimeoutDetails;
    UsDrvErrorStatus sensorFwVersionIncompatible;
    UsDrvErrorStatus sensorHwVersionIncompatible;
    UsDrvErrorStatus sensorInitFiringSchemeSendFail;
    UsDrvErrorStatus sensorInitReceivingSchemeSendFail;
    UsDrvErrorStatus sensorInitBusError;
    UsDrvErrorStatus sensorInitScodeIncorrect;
    UsDrvErrorStatus sensorPdcmCommTimeout;
    UsDrvErrorStatus unexpectedMeasModeRecfg;
  };

  inline ::us_drv::UsDrvSensorSwErrors createUsDrvSensorSwErrors()
  {
    UsDrvSensorSwErrors m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvSensorSwErrors));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvSensorSwErrors create_default()
  {
      return ::us_drv::createUsDrvSensorSwErrors();
  }
}


#endif // US_DRV_US_DRV_SENSOR_SW_ERRORS_H_
