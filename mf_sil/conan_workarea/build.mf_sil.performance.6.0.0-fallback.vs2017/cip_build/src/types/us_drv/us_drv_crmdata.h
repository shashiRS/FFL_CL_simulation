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

#ifndef US_DRV_US_DRV_CRMDATA_H_
#define US_DRV_US_DRV_CRMDATA_H_

#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvCRMData
  {
    uint8 commandStatus;
    uint8 physicalAddress;
    uint8 extendedData;
    uint8 registerData;
    uint8 crc;
  };

  inline ::us_drv::UsDrvCRMData createUsDrvCRMData()
  {
    UsDrvCRMData m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvCRMData));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvCRMData create_default()
  {
      return ::us_drv::createUsDrvCRMData();
  }
}


#endif // US_DRV_US_DRV_CRMDATA_H_
