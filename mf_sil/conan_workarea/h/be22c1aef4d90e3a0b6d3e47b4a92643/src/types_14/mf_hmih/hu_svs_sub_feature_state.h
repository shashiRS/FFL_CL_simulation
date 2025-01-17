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

#ifndef MF_HMIH_HU_SVS_SUB_FEATURE_STATE_H_
#define MF_HMIH_HU_SVS_SUB_FEATURE_STATE_H_

#include "Platform_Types.h"

namespace mf_hmih
{
  ///current status of the SVS  features
  enum class HuSVsSubFeatureState : uint8
  {
      UNUSED = 0U,
      SVS_2D_ACTIVE = 1U,
      SVS_3D_ACTIVE = 2U,
      SVS_FRONT_CORNER_ACTIVE = 3U,
      SVS_REAR_CORNER_ACTIVE = 4U,
  };
} // namespace mf_hmih
#endif // MF_HMIH_HU_SVS_SUB_FEATURE_STATE_H_
