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

#ifndef AP_PSM_APP_PPCPARKING_MODE_C_H_
#define AP_PSM_APP_PPCPARKING_MODE_C_H_

#include "Platform_Types.h"

typedef uint8 AP_PSM_APP_PPCParkingMode;

#define AP_PSM_APP_PPCPARKING_MODE_PARKING_MODE_NOT_VALID 0U
#define AP_PSM_APP_PPCPARKING_MODE_PARK_IN_FULL_MANEUVERING_AREA 1U
#define AP_PSM_APP_PPCPARKING_MODE_PARK_IN_RESTRICTED_MANEUVERING_AREA 13U
#define AP_PSM_APP_PPCPARKING_MODE_PARK_OUT_UNTIL_CRITICAL_POINT_REACHED 2U
#define AP_PSM_APP_PPCPARKING_MODE_PARK_OUT_TO_TARGET_POSE 14U
#define AP_PSM_APP_PPCPARKING_MODE_GARAGE_PARKING_IN 3U
#define AP_PSM_APP_PPCPARKING_MODE_GARAGE_PARKING_OUT 4U
#define AP_PSM_APP_PPCPARKING_MODE_TRAINED_PARKING_TRAIN 5U
#define AP_PSM_APP_PPCPARKING_MODE_TRAINED_PARKING_EXEC 6U
#define AP_PSM_APP_PPCPARKING_MODE_REMOTE_MANEUVERING 7U
#define AP_PSM_APP_PPCPARKING_MODE_MEMORY_PARKING_TRAIN 8U
#define AP_PSM_APP_PPCPARKING_MODE_MEMORY_PARKING_EXEC 9U
#define AP_PSM_APP_PPCPARKING_MODE_UNDO_MANEUVER 10U
#define AP_PSM_APP_PPCPARKING_MODE_REVERSE_ASSIST_ACTIVE 11U
#define AP_PSM_APP_PPCPARKING_MODE_REMOTE_SELF_TEST 12U


#endif // AP_PSM_APP_PPCPARKING_MODE_C_H_
