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

#ifndef US_DRV_US_DRV_CRM_STATUS_COUNTS_H_
#define US_DRV_US_DRV_CRM_STATUS_COUNTS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvCrmStatusCounts
  {
    uint32 dataOk;
    uint32 overWritten;
    uint32 crmError;
    uint32 reserved;
    uint32 lengthError;
    uint32 timingError;
    uint32 timeoutError;
    uint32 crcError;
  };

  inline ::us_drv::UsDrvCrmStatusCounts createUsDrvCrmStatusCounts()
  {
    UsDrvCrmStatusCounts m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvCrmStatusCounts));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvCrmStatusCounts create_default()
  {
      return ::us_drv::createUsDrvCrmStatusCounts();
  }
}


#endif // US_DRV_US_DRV_CRM_STATUS_COUNTS_H_