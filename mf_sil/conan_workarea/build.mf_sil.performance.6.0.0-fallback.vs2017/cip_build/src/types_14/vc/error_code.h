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

#ifndef VC_ERROR_CODE_H_
#define VC_ERROR_CODE_H_

#include "Platform_Types.h"

namespace vc
{
  enum class ErrorCode : uint16
  {
      VIS_ERRORCODE_OK = 0U,
      VIS_ERRORCODE_FAILURE = 1U,
      VIS_ERRORCODE_OVERLAY_FAILED = 2U,
      VIS_ERRORCODE_VIDEO_FAILED = 3U,
      VIS_ERRORCODE_INVALID_VIEW = 4U,
      VIS_ERRORCODE_NOTACTIVATED = 5U,
      VIS_ERRORCODE_DEACTIVATED = 6U,
      VIS_ERRORCODE_COUNT = 7U,
      VIS_ERRORCODE_MAX = 256U,
  };
} // namespace vc
#endif // VC_ERROR_CODE_H_
