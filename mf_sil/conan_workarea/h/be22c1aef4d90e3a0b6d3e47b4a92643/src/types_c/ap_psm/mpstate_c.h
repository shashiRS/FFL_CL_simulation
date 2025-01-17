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

#ifndef AP_PSM_MPSTATE_C_H_
#define AP_PSM_MPSTATE_C_H_

#include "Platform_Types.h"

///State of the Memory Parking.
typedef uint8 AP_PSM_MPState;

#define AP_PSM_MPSTATE_MP_INACTIVE 0U
#define AP_PSM_MPSTATE_MP_TRAIN_SCAN 1U
#define AP_PSM_MPSTATE_MP_TRAIN_AVG 2U
#define AP_PSM_MPSTATE_MP_TRAIN_FINISHED 3U
#define AP_PSM_MPSTATE_MP_SILENT_MAP_LOC 4U
#define AP_PSM_MPSTATE_MP_RECALL_SCAN 5U
#define AP_PSM_MPSTATE_MP_RECALL_AVG 6U
#define AP_PSM_MPSTATE_MP_RECALL_FINISHED 7U
#define AP_PSM_MPSTATE_MP_PAUSE 8U


#endif // AP_PSM_MPSTATE_C_H_
