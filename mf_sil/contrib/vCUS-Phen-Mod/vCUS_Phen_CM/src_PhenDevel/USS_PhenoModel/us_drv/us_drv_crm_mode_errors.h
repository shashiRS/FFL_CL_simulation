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

#ifndef US_DRV_US_DRV_CRM_MODE_ERRORS_H_
#define US_DRV_US_DRV_CRM_MODE_ERRORS_H_

#include "us_drv/us_drv_error_status.h"
#include "Platform_Types.h"
#include "eco/memset.h"


namespace us_drv
{

  struct UsDrvCrmModeErrors
  {
    UsDrvErrorStatus crmCmdFailure;
    uint8 crmCmdFailureDetails;
    UsDrvErrorStatus dataEarlyError;
    UsDrvErrorStatus dataLateError;
    UsDrvErrorStatus symbolCountError;
    UsDrvErrorStatus crcError;
    UsDrvErrorStatus spiCmdSequenceFailure;
    UsDrvErrorStatus symbolCodingError;
    UsDrvErrorStatus dsiPinUndervoltage;
    UsDrvErrorStatus clkRefError;
  };

  inline ::us_drv::UsDrvCrmModeErrors createUsDrvCrmModeErrors()
  {
    UsDrvCrmModeErrors m;
    (void)::eco::memset(&m, 0U, sizeof(UsDrvCrmModeErrors));
    return m;
  }

} // namespace us_drv

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::us_drv::UsDrvCrmModeErrors create_default()
  {
      return ::us_drv::createUsDrvCrmModeErrors();
  }
}


#endif // US_DRV_US_DRV_CRM_MODE_ERRORS_H_
