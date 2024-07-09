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

#ifndef AP_TP_PLANNING_FAIL_REASON_C_H_
#define AP_TP_PLANNING_FAIL_REASON_C_H_

#include "Platform_Types.h"

///
typedef uint8 AP_TP_PlanningFailReason;

#define AP_TP_PLANNING_FAIL_REASON_PFR_NONE 0U
#define AP_TP_PLANNING_FAIL_REASON_PFR_TARGET_POSE_LOST 1U
#define AP_TP_PLANNING_FAIL_REASON_PFR_PARKING_BOX_LOST 2U
#define AP_TP_PLANNING_FAIL_REASON_PFR_INPUT_CORRUPTED 3U
#define AP_TP_PLANNING_FAIL_REASON_PFR_REPLAN_FAIL 4U
#define AP_TP_PLANNING_FAIL_REASON_MAX_NUM_PLANNING_FAIL_TYPES 5U


#endif // AP_TP_PLANNING_FAIL_REASON_C_H_
