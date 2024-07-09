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

#ifndef AP_PSM_APP_PPCSTATE_C_H_
#define AP_PSM_APP_PPCSTATE_C_H_

#include "Platform_Types.h"

///State of the parking procedure state machine
typedef uint8 AP_PSM_APP_PPCState;

#define AP_PSM_APP_PPCSTATE_PPC_INACTIVE 0U
#define AP_PSM_APP_PPCSTATE_PPC_BEHAVIOR_INACTIVE 1U
#define AP_PSM_APP_PPCSTATE_PPC_SCANNING_INACTIVE 2U
#define AP_PSM_APP_PPCSTATE_PPC_SCANNING_IN 3U
#define AP_PSM_APP_PPCSTATE_PPC_SCANNING_OUT 4U
#define AP_PSM_APP_PPCSTATE_PPC_SCANNING_RM 5U
#define AP_PSM_APP_PPCSTATE_PPC_PARKING_INACTIVE 6U
#define AP_PSM_APP_PPCSTATE_PPC_PREPARE_PARKING 7U
#define AP_PSM_APP_PPCSTATE_PPC_PERFORM_PARKING 8U
#define AP_PSM_APP_PPCSTATE_PPC_PARKING_PAUSE 9U
#define AP_PSM_APP_PPCSTATE_PPC_PARKING_FAILED 10U
#define AP_PSM_APP_PPCSTATE_PPC_OFFER_HANDOVER 11U
#define AP_PSM_APP_PPCSTATE_PPC_SUCCESS 12U
#define AP_PSM_APP_PPCSTATE_PPC_PARKING_CANCELED 13U
#define AP_PSM_APP_PPCSTATE_PPC_IRREVERSIBLE_ERROR 14U
#define AP_PSM_APP_PPCSTATE_PPC_REVERSIBLE_ERROR 15U
#define AP_PSM_APP_PPCSTATE_PPC_DEMO_OFF 23U
#define AP_PSM_APP_PPCSTATE_PPC_SCANNING_GP 24U


#endif // AP_PSM_APP_PPCSTATE_C_H_
