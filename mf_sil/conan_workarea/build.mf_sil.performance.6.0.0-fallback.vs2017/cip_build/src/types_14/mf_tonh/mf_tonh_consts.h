// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_14\struct.h.template!

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

#ifndef MF_TONH_MF_TONH_CONSTS_H_
#define MF_TONH_MF_TONH_CONSTS_H_

#include "Platform_Types.h"


namespace mf_tonh
{

  /// TONH consts
  struct MF_TONH_Consts
  {
    static constexpr uint8 NUM_SPEAKERS = 2U;
    static constexpr uint8 NUM_MTS_DEBUG_FREESPACE_TONH = 10U;
  };

} // namespace mf_tonh

#endif // MF_TONH_MF_TONH_CONSTS_H_
