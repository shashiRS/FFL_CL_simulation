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

#ifndef SI_SLOT_COORDINATES_M_SERIALIZABLE_H_
#define SI_SLOT_COORDINATES_M_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "cml/vec2_df_pod.h"


namespace si
{

  struct SlotCoordinates_mSerializable
  {
    ::lsm_geoml::size_type actualSize{};
    ::cml::Vec2Df_POD array[4]{};
  };

} // namespace si

#endif // SI_SLOT_COORDINATES_M_SERIALIZABLE_H_
