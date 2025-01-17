// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\enum.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP14_TYPES_USED

#ifndef ECO_DIAGNOSIS_EVENT_STATUS_H_
#define ECO_DIAGNOSIS_EVENT_STATUS_H_

#include "Platform_Types.h"

namespace eco
{
  ///values: enum { DIAGNOSIS_EVENT_STATUS_PASSED=0,DIAGNOSIS_EVENT_STATUS_FAILED=1,
  ///DIAGNOSIS_EVENT_STATUS_PREPASSED=2,DIAGNOSIS_EVENT_STATUS_PREFAILED=3}
  ///@range{0,255}
  enum class DiagnosisEventStatus : uint8
  {
      DIAGNOSIS_EVENT_STATUS_PASSED = 0U,
      DIAGNOSIS_EVENT_STATUS_FAILED = 1U,
      DIAGNOSIS_EVENT_STATUS_PREPASSED = 2U,
      DIAGNOSIS_EVENT_STATUS_PREFAILED = 3U,
  };
} // namespace eco
#endif // ECO_DIAGNOSIS_EVENT_STATUS_H_
