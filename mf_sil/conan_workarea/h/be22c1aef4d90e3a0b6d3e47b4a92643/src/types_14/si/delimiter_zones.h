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

#ifndef SI_DELIMITER_ZONES_H_
#define SI_DELIMITER_ZONES_H_

#include "si/quadrilateral_serializable.h"
#include "Platform_Types.h"


namespace si
{

  /// @brief Structure containing the different delimiting bounding boxes.
  struct DelimiterZones
  {
    ///@brief Bounding box describing the curb zone.
    QuadrilateralSerializable curbZone{};
    ///@brief Bounding box describing the road zone.
    QuadrilateralSerializable roadZone{};
    ///@brief Bounding box describing the left zone.
    QuadrilateralSerializable leftZone{};
    ///@brief Bounding box describing the right zone.
    QuadrilateralSerializable rightZone{};
    ///@brief Bounding box describing the inside zone.
    QuadrilateralSerializable insideZone{};
    ///@brief Bounding box for summation of all of the above.
    QuadrilateralSerializable all{};
    ///@range{0,255}
    ///@unit{nu}
    ///@brief ID of the specific slot.
    uint8 slotId_nu{255U};
  };

} // namespace si

#endif // SI_DELIMITER_ZONES_H_
