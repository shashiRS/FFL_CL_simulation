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

#ifndef ECO_DIAGNOSIS_EVENT_STATUS_C_H_
#define ECO_DIAGNOSIS_EVENT_STATUS_C_H_

#include "Platform_Types.h"

///values: enum { DIAGNOSIS_EVENT_STATUS_PASSED=0,DIAGNOSIS_EVENT_STATUS_FAILED=1,
///DIAGNOSIS_EVENT_STATUS_PREPASSED=2,DIAGNOSIS_EVENT_STATUS_PREFAILED=3}
///@range{0,255}
typedef uint8 ECO_DiagnosisEventStatus;

#define ECO_DIAGNOSIS_EVENT_STATUS_DIAGNOSIS_EVENT_STATUS_PASSED 0U
#define ECO_DIAGNOSIS_EVENT_STATUS_DIAGNOSIS_EVENT_STATUS_FAILED 1U
#define ECO_DIAGNOSIS_EVENT_STATUS_DIAGNOSIS_EVENT_STATUS_PREPASSED 2U
#define ECO_DIAGNOSIS_EVENT_STATUS_DIAGNOSIS_EVENT_STATUS_PREFAILED 3U


#endif // ECO_DIAGNOSIS_EVENT_STATUS_C_H_