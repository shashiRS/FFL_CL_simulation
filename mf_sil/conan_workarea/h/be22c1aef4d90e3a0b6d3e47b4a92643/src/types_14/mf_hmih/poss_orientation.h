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

#ifndef MF_HMIH_POSS_ORIENTATION_H_
#define MF_HMIH_POSS_ORIENTATION_H_

#include "Platform_Types.h"

namespace mf_hmih
{
  ///Indicates wether orientation of parking space is certain or not.
  enum class PossOrientation : uint8
  {
      POSS_ORI_CERTAIN = 0U,
      POSS_ORI_UNCERTAIN = 1U,
  };
} // namespace mf_hmih
#endif // MF_HMIH_POSS_ORIENTATION_H_
