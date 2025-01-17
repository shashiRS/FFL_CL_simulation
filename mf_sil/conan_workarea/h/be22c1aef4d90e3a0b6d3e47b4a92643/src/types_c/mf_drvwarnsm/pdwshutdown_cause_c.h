//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\enum.h.template!

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

#ifndef MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_C_H_
#define MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_C_H_

#include "Platform_Types.h"

///The reason why  PDW system state is off
typedef uint8 MF_DRVWARNSM_PDWShutdownCause;

#define MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_PDW_NONE_SYS_ACTIVE 0U
#define MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_PDW_SHUTDOWN_BY_BUTTON 1U
#define MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_PDW_SHUTDOWN_BY_DISENGAGED_R_GEAR 2U
#define MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_PDW_SHUTDOWN_BY_SPEED 3U


#endif // MF_DRVWARNSM_PDWSHUTDOWN_CAUSE_C_H_
