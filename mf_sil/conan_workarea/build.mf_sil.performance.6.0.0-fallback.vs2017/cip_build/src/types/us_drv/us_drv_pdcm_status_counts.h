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

#ifndef US_DRV_US_DRV_PDCM_STATUS_COUNTS_H_
#define US_DRV_US_DRV_PDCM_STATUS_COUNTS_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvPdcmStatusCounts
  {
    uint32 dataOk;
    uint32 overWritten;
    uint32 pdcmError;
    uint32 outOfSync;
    uint32 lengthError;
    uint32 timingError;
    uint32 timeoutError;
    uint32 crcError;
  };

  inline ::us_drv::UsDrvPdcmStatusCounts createUsDrvPdcmStatusCounts()
  {
    UsDrvPdcmStatusCounts m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvPdcmStatusCounts));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvPdcmStatusCounts create_default()
  {
      return ::us_drv::createUsDrvPdcmStatusCounts();
  }
}


#endif // US_DRV_US_DRV_PDCM_STATUS_COUNTS_H_
