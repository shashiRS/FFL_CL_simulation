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

#ifndef MF_HMIH_APUSER_ACTION_HEAD_UNIT_H_
#define MF_HMIH_APUSER_ACTION_HEAD_UNIT_H_

#include "Platform_Types.h"

namespace mf_hmih
{
  ///User interaction with HMI handler (Head Unit)
  enum class APUserActionHeadUnit : uint8
  {
      AP_NO_USER_ACTION = 0U,
      AP_TAP_ON_START_SELECTION = 1U,
      AP_TAP_ON_START_PARKING = 2U,
      AP_TAP_ON_INTERRUPT = 3U,
      AP_TAP_ON_CONTINUE = 4U,
      AP_TAP_ON_UNDO = 5U,
      AP_TAP_ON_CANCEL = 6U,
      AP_TAP_ON_REDO = 7U,
      AP_TAP_ON_START_REMOTE_PARKING = 8U,
      AP_TAP_ON_SWITCH_DIRECTION = 9U,
      AP_TAP_ON_SWITCH_ORIENTATION = 10U,
      AP_TAP_ON_PREVIOUS_SCREEN = 11U,
      AP_TOGGLE_AP_ACTIVE = 12U,
      AP_TAP_ON_FULLY_AUTOMATED_PARKING = 13U,
      AP_TAP_ON_SEMI_AUTOMATED_PARKING = 14U,
      AP_TAP_ON_START_KEY_PARKING = 15U,
      AP_TAP_ON_GP = 16U,
      AP_TAP_ON_RM = 17U,
      AP_TAP_SWITCH_TO_REMOTE_APP = 18U,
      AP_TAP_SWITCH_TO_REMOTE_KEY = 19U,
      AP_TAP_ON_EXPLICIT_SCANNING = 20U,
      AP_TAP_ON_REVERSE_ASSIST = 21U,
      AP_TAP_ON_USER_SLOT_DEFINE = 22U,
      AP_TAP_ON_MEMORY_PARKING = 23U,
      AP_TAP_ON_USER_SLOT_REFINE = 24U,
      AP_TAP_ON_USER_SLOT_SAVE = 25U,
      AP_TAP_ON_USER_SLOT_CLOSE = 26U,
      AP_TAP_ON_USER_SLOT_DELETE = 27U,
      AP_TAP_ON_CROSS = 28U,
  };
} // namespace mf_hmih
#endif // MF_HMIH_APUSER_ACTION_HEAD_UNIT_H_