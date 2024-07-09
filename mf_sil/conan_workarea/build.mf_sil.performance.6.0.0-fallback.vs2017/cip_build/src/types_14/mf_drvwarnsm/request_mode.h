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

#ifndef MF_DRVWARNSM_REQUEST_MODE_H_
#define MF_DRVWARNSM_REQUEST_MODE_H_

#include "Platform_Types.h"

namespace mf_drvwarnsm
{
  ///The PDW request mode to the core state machines
  enum class RequestMode : uint8
  {
      REQ_NO_REQUEST = 0U,
      REQ_INIT = 1U,
      REQ_INACTIVE = 2U,
      REQ_ACTIVE = 3U,
      REQ_FAILURE = 4U,
      REQ_NUM = 5U,
  };
} // namespace mf_drvwarnsm
#endif // MF_DRVWARNSM_REQUEST_MODE_H_