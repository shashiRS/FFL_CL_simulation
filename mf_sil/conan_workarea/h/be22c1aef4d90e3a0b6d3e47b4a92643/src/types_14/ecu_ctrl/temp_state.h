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

#ifndef ECU_CTRL_TEMP_STATE_H_
#define ECU_CTRL_TEMP_STATE_H_

#include "Platform_Types.h"

namespace ecu_ctrl
{
  enum class TempState : uint8
  {
      ///TEMP_STATE_INIT data or struct is not valid
      TEMP_STATE_INIT = 0U,
      ///TEMP_STATE_NORMAL data is valid for other usage
      TEMP_STATE_NORMAL = 1U,
      ///TEMP_STATE_OVER_TEMP data is invalid or out of range, use with care
      TEMP_STATE_OVER_TEMP = 2U,
      ///TEMP_STATE_UNDER_TEMP data in timeout state
      TEMP_STATE_UNDER_TEMP = 3U,
      ///TEMP_STATE_UNKNOWN data in error state
      TEMP_STATE_UNKNOWN = 128U,
  };
} // namespace ecu_ctrl
#endif // ECU_CTRL_TEMP_STATE_H_
