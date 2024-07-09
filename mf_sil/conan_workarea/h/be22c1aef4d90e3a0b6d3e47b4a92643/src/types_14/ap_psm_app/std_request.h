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

#ifndef AP_PSM_APP_STD_REQUEST_H_
#define AP_PSM_APP_STD_REQUEST_H_

#include "Platform_Types.h"

namespace ap_psm_app
{
  enum class StdRequest : uint8
  {
      STD_REQ_NO_REQUEST = 0U,
      STD_REQ_INIT = 1U,
      STD_REQ_SCAN = 2U,
      STD_REQ_PREPARE = 3U,
      STD_REQ_DEGRADE = 4U,
      STD_REQ_START = 5U,
      STD_REQ_PAUSE = 6U,
      STD_REQ_SECURE = 7U,
      STD_REQ_FINISH = 8U,
      STD_REQ_ERROR = 9U,
  };
} // namespace ap_psm_app
#endif // AP_PSM_APP_STD_REQUEST_H_
