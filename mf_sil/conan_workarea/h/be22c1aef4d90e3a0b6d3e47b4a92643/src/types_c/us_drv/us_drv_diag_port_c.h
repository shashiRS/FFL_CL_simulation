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

#ifndef US_DRV_US_DRV_DIAG_PORT_C_H_
#define US_DRV_US_DRV_DIAG_PORT_C_H_

#include "eco/algo_interface_version_number_c.h"
#include "eco/signal_header_c.h"
#include "us_drv/us_drv_variant_data_c.h"
#include "us_drv/us_drv_sw_errors_c.h"
#include "us_drv/us_drv_asic_errors_c.h"
#include "us_drv/us_drv_sensor_errors_c.h"
#include "eco/memset_c.h"

typedef struct
{
    ECO_AlgoInterfaceVersionNumber uiVersionNumber;
    ECO_SignalHeader sSigHeader;
    US_DRV_UsDrvVariantData usDriverVariantData;
    US_DRV_UsDrvSwErrors usDriverSwErrors;
    US_DRV_UsDrvAsicErrors asicErrors[2];
    US_DRV_UsDrvSensorErrors sensorErrors[18];
} US_DRV_UsDrvDiagPort;

inline US_DRV_UsDrvDiagPort create_US_DRV_UsDrvDiagPort(void)
{
  US_DRV_UsDrvDiagPort m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.sSigHeader = create_ECO_SignalHeader();
  m.usDriverVariantData = create_US_DRV_UsDrvVariantData();
  m.usDriverSwErrors = create_US_DRV_UsDrvSwErrors();
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.asicErrors) / sizeof(m.asicErrors[0])); ++i)
    {
      m.asicErrors[i] = create_US_DRV_UsDrvAsicErrors();
    }
  }
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.sensorErrors) / sizeof(m.sensorErrors[0])); ++i)
    {
      m.sensorErrors[i] = create_US_DRV_UsDrvSensorErrors();
    }
  }
  return m;
}

#endif // US_DRV_US_DRV_DIAG_PORT_C_H_
