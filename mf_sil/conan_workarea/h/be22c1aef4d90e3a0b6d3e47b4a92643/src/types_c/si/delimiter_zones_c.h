//Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types_c\struct.h.template!

#ifdef ECO_CPP_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++14 types mixed
  #endif
#endif
#define ECO_C_TYPES_USED

#ifndef SI_DELIMITER_ZONES_C_H_
#define SI_DELIMITER_ZONES_C_H_

#include "si/quadrilateral_serializable_c.h"
#include "Platform_Types.h"
#include "eco/memset_c.h"

/// @brief Structure containing the different delimiting bounding boxes.
typedef struct
{
    ///@brief Bounding box describing the curb zone.
    SI_QuadrilateralSerializable curbZone;
    ///@brief Bounding box describing the road zone.
    SI_QuadrilateralSerializable roadZone;
    ///@brief Bounding box describing the left zone.
    SI_QuadrilateralSerializable leftZone;
    ///@brief Bounding box describing the right zone.
    SI_QuadrilateralSerializable rightZone;
    ///@brief Bounding box describing the inside zone.
    SI_QuadrilateralSerializable insideZone;
    ///@brief Bounding box for summation of all of the above.
    SI_QuadrilateralSerializable all;
    ///@range{0,255}
    ///@unit{nu}
    ///@brief ID of the specific slot.
    uint8 slotId_nu;
} SI_DelimiterZones;

inline SI_DelimiterZones create_SI_DelimiterZones(void)
{
  SI_DelimiterZones m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.curbZone = create_SI_QuadrilateralSerializable();
  m.roadZone = create_SI_QuadrilateralSerializable();
  m.leftZone = create_SI_QuadrilateralSerializable();
  m.rightZone = create_SI_QuadrilateralSerializable();
  m.insideZone = create_SI_QuadrilateralSerializable();
  m.all = create_SI_QuadrilateralSerializable();
  m.slotId_nu = 255U;
  return m;
}

#endif // SI_DELIMITER_ZONES_C_H_
