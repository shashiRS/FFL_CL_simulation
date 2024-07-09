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

#ifndef US_DRV_US_DRV_CRM_MODE_ERRORS_C_H_
#define US_DRV_US_DRV_CRM_MODE_ERRORS_C_H_

#include "us_drv/us_drv_error_status_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

typedef struct
{
    US_DRV_UsDrvErrorStatus crmCmdFailure;
    uint8 crmCmdFailureDetails;
    US_DRV_UsDrvErrorStatus dataEarlyError;
    US_DRV_UsDrvErrorStatus dataLateError;
    US_DRV_UsDrvErrorStatus symbolCountError;
    US_DRV_UsDrvErrorStatus crcError;
    US_DRV_UsDrvErrorStatus spiCmdSequenceFailure;
    US_DRV_UsDrvErrorStatus symbolCodingError;
    US_DRV_UsDrvErrorStatus dsiPinUndervoltage;
    US_DRV_UsDrvErrorStatus clkRefError;
} US_DRV_UsDrvCrmModeErrors;

inline US_DRV_UsDrvCrmModeErrors create_US_DRV_UsDrvCrmModeErrors(void)
{
  US_DRV_UsDrvCrmModeErrors m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // US_DRV_US_DRV_CRM_MODE_ERRORS_C_H_
