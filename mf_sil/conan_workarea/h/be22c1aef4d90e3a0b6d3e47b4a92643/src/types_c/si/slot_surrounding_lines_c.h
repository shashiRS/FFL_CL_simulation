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

#ifndef SI_SLOT_SURROUNDING_LINES_C_H_
#define SI_SLOT_SURROUNDING_LINES_C_H_

#include "si/line_segment_serializable_c.h"
#include "eco/memset_c.h"

/// @brief Aggregate structure for the lines surrounding the slot.
typedef struct
{
    ///@brief Describes a surrounding line on the left side.
    SI_LineSegmentSerializable leftLine;
    ///@brief Describes a surrounding line on the right side.
    SI_LineSegmentSerializable rightLine;
    ///@brief Describes a surrounding line on the road side.
    SI_LineSegmentSerializable roadLine;
    ///@brief Describes a surrounding line on the curb side.
    SI_LineSegmentSerializable curbLine;
} SI_SlotSurroundingLines;

inline SI_SlotSurroundingLines create_SI_SlotSurroundingLines(void)
{
  SI_SlotSurroundingLines m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.leftLine = create_SI_LineSegmentSerializable();
  m.rightLine = create_SI_LineSegmentSerializable();
  m.roadLine = create_SI_LineSegmentSerializable();
  m.curbLine = create_SI_LineSegmentSerializable();
  return m;
}

#endif // SI_SLOT_SURROUNDING_LINES_C_H_
