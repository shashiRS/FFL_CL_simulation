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

#ifndef MF_HMIH_APUSER_ACTION_REMOTE_DEVICE_H_
#define MF_HMIH_APUSER_ACTION_REMOTE_DEVICE_H_

#include "Platform_Types.h"

namespace mf_hmih
{
  ///User interaction with HMI handler (Remote Parking App)
  enum class APUserActionRemoteDevice : uint8
  {
      AP_REM_NO_USER_ACTION = 0U,
      AP_REM_APP_STARTED = 1U,
      AP_REM_APP_CLOSED = 2U,
      AP_REM_TAP_ON_START_PARKING = 3U,
      AP_REM_TAP_ON_INTERRUPT = 4U,
      AP_REM_TAP_ON_CONTINUE = 5U,
      AP_REM_TAP_ON_UNDO = 6U,
      AP_REM_TAP_ON_CANCEL = 7U,
      AP_REM_TAP_ON_REDO = 8U,
      AP_REM_TAP_ON_PARK_IN = 9U,
      AP_REM_TAP_ON_PARK_OUT = 10U,
      AP_REM_TAP_ON_REM_MAN = 11U,
      AP_REM_TAP_ON_REM_SV = 12U,
      AP_REM_TAP_ON_REM_FWD = 13U,
      AP_REM_TAP_ON_REM_BWD = 14U,
      AP_REM_TAP_ON_PREVIOUS_SCREEN = 15U,
      AP_REM_TAP_ON_GP = 16U,
      AP_REM_TAP_SWITCH_TO_HEAD_UNIT = 17U,
  };
} // namespace mf_hmih
#endif // MF_HMIH_APUSER_ACTION_REMOTE_DEVICE_H_
