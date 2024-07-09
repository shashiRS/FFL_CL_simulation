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

#ifndef MF_MEMPARK_STORING_STATUS_H_
#define MF_MEMPARK_STORING_STATUS_H_

#include "Platform_Types.h"

namespace mf_mempark
{
  ///The status of the storing, loading and deleting data.
  enum class StoringStatus : uint8
  {
      STORING_STATUS_INIT = 0U,
      STORING_STATUS_IDLE = 1U,
      STORING_STATUS_ONGOING = 2U,
      STORING_STATUS_SUCCESS = 3U,
      STORING_STATUS_FAILED = 4U,
      STORING_STATUS_NOT_VALID = 5U,
  };
} // namespace mf_mempark
#endif // MF_MEMPARK_STORING_STATUS_H_
