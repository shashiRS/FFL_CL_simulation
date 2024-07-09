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

#ifndef MF_LSCA_MF_LSCA_CONSTS_H_
#define MF_LSCA_MF_LSCA_CONSTS_H_

#include "Platform_Types.h"


namespace mf_lsca
{

  struct MF_LSCA_Consts
  {
    static constexpr uint8 MAX_BODY_SHAPE_SIZE = 16U;
    static constexpr uint8 MAX_BODY_INDICES_SIZE = 17U;
    static constexpr uint8 MAX_WHEEL_SHAPE_SIZE = 4U;
    static constexpr uint8 MAX_HITCH_SHAPE_SIZE = 4U;
    static constexpr uint8 MAX_MIRROR_SHAPE_SIZE = 4U;
    static constexpr uint8 MAX_SIMPLE_BODY_SHAPE_SIZE = 4U;
    static constexpr uint8 MAX_STATIC_OBJECT_SIZE_NU = 16U;
    static constexpr uint8 MAX_DYNAMIC_OBJECT_SIZE_NU = 4U;
    static constexpr uint8 MAX_DYNAMIC_OBJECT_PREDICTION_SIZE_NU = 6U;
    static constexpr uint8 MAX_STATIC_OBJECTS_BRAKE_NU = 32U;
    static constexpr uint8 MAX_DYNAMIC_OBJECTS_BRAKE_NU = 10U;
    static constexpr uint8 STATIC_BRAKE_ROI_SIZE = 4U;
    static constexpr uint8 DYNAMIC_BRAKE_ROI_SIZE = 4U;
  };

} // namespace mf_lsca

#endif // MF_LSCA_MF_LSCA_CONSTS_H_