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

#ifndef MF_LSCA_LSCA_STATE_H_
#define MF_LSCA_LSCA_STATE_H_

#include "Platform_Types.h"

namespace mf_lsca
{
  enum class LSCA_STATE : uint8
  {
      LSCA_STATE_DEACTIVATED = 0U,
      LSCA_STATE_ACTIVATED = 1U,
      LSCA_STATE_INTERVENTION = 2U,
      LSCA_STATE_ERROR = 3U,
  };
} // namespace mf_lsca
#endif // MF_LSCA_LSCA_STATE_H_
