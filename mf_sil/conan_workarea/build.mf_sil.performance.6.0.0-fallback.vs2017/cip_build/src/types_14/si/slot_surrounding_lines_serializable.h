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

#ifndef SI_SLOT_SURROUNDING_LINES_SERIALIZABLE_H_
#define SI_SLOT_SURROUNDING_LINES_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "si/slot_surrounding_lines.h"


namespace si
{

  /// @brief Slot surrounding lines serializable.
  struct SlotSurroundingLinesSerializable
  {
    ///@range{0,AP_Common.AP_COMMON_TYPES_Consts.AP_G_MAX_NUM_PARKING_BOXES_NU}
    ///@unit{nu}
    ///@brief Describes how many surrounding lines were already added to the array.
    ::lsm_geoml::size_type actualSize{0U};
    ///@brief Array of slot surrounding lines.
    SlotSurroundingLines array[6]{};
  };

} // namespace si

#endif // SI_SLOT_SURROUNDING_LINES_SERIALIZABLE_H_
