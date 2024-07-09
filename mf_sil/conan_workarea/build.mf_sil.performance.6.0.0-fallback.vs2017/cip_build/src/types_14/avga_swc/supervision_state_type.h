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

#ifndef AVGA_SWC_SUPERVISION_STATE_TYPE_H_
#define AVGA_SWC_SUPERVISION_STATE_TYPE_H_

#include "Platform_Types.h"

namespace avga_swc
{
  enum class SupervisionStateType : uint8
  {
      AVGA_OFF = 0U,
      AVGA_ON = 1U,
      AVGA_INTERVENING = 2U,
  };
} // namespace avga_swc
#endif // AVGA_SWC_SUPERVISION_STATE_TYPE_H_