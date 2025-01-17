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

#ifndef CFG_MGR_ECU_CODING_STATE_H_
#define CFG_MGR_ECU_CODING_STATE_H_

#include "Platform_Types.h"

namespace cfg_mgr
{
  enum class EcuCodingState : uint8
  {
      ///CODING_STATE_INIT data or struct is not valid
      CODING_STATE_INIT = 0U,
      ///CODING_STATE_VALID data is valid for other usage
      CODING_STATE_VALID = 1U,
      ///CODING_STATE_DEFAULT data is invalid or out of range, usage with care
      CODING_STATE_DEFAULT = 2U,
      ///CODING_STATE_UNKNOWN data in error state
      CODING_STATE_UNKNOWN = 128U,
  };
} // namespace cfg_mgr
#endif // CFG_MGR_ECU_CODING_STATE_H_
