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

#ifndef MF_MEMPARK_USER_UPDATE_REQUEST_STATUS_H_
#define MF_MEMPARK_USER_UPDATE_REQUEST_STATUS_H_

#include "Platform_Types.h"

namespace mf_mempark
{
  ///Current status of the slot update request from user. It indicates
  ///if the update request was successful and if the last request
  ///pushed the update to maximum allowed limits.
  enum class UserUpdateRequestStatus : uint16
  {
      UPDATE_STATUS_INIT = 0U,
      UPDATE_STATUS_SUCCESS = 1U,
      UPDATE_STATUS_SUCCESS_MAX_C_ANGLE = 2U,
      UPDATE_STATUS_SUCCESS_MAX_CC_ANGLE = 3U,
      UPDATE_STATUS_SUCCESS_MAX_LONG_POS_UP = 4U,
      UPDATE_STATUS_SUCCESS_MAX_LONG_POS_DOWN = 5U,
      UPDATE_STATUS_SUCCESS_MAX_LAT_POS_LEFT = 6U,
      UPDATE_STATUS_SUCCESS_MAX_LAT_POS_RIGHT = 7U,
      UPDATE_STATUS_SUCCESS_MAX_ATTEMPTS = 8U,
      UPDATE_STATUS_FAILED = 9U,
  };
} // namespace mf_mempark
#endif // MF_MEMPARK_USER_UPDATE_REQUEST_STATUS_H_