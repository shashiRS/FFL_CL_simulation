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

#ifndef US_DRV_US_DRV_DEBUG_PORT_BUS_H_
#define US_DRV_US_DRV_DEBUG_PORT_BUS_H_

#include "Platform_Types.h"
#include "us_drv/us_drv_crm_status_counts.h"
#include "us_drv/us_drv_pdcm_status_counts.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvDebugPortBus
  {
    uint32 crmCount;
    uint32 crmErrorCount;
    UsDrvCrmStatusCounts crmStatusCounts;
    uint32 pdcmCount;
    uint32 pdcmErrorCount;
    uint32 pdcmSizeMismatchCount;
    UsDrvPdcmStatusCounts pdcmStatusCounts;
  };

  inline ::us_drv::UsDrvDebugPortBus createUsDrvDebugPortBus()
  {
    UsDrvDebugPortBus m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvDebugPortBus));
    m.crmStatusCounts = createUsDrvCrmStatusCounts();
    m.pdcmStatusCounts = createUsDrvPdcmStatusCounts();
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvDebugPortBus create_default()
  {
      return ::us_drv::createUsDrvDebugPortBus();
  }
}


#endif // US_DRV_US_DRV_DEBUG_PORT_BUS_H_
