// Attention, this file is generated by Cobolt from template: D:\.bbs_conan\c029a0\1\codegen\templates\types\struct.h.template!

#ifdef ECO_C_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C and C++ types mixed
  #endif
#endif
#ifdef ECO_CPP14_TYPES_USED
  #ifndef ECO_ALLOW_INTERFACE_MIXING
    #error eco C++ and C++14 types mixed
  #endif
#endif
#define ECO_CPP_TYPES_USED

#ifndef SI_SLOT_SURROUNDING_LINES_H_
#define SI_SLOT_SURROUNDING_LINES_H_

#include "si/line_segment_serializable.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Aggregate structure for the lines surrounding the slot.
  struct SlotSurroundingLines
  {
    ///@brief Describes a surrounding line on the left side.
    LineSegmentSerializable leftLine;
    ///@brief Describes a surrounding line on the right side.
    LineSegmentSerializable rightLine;
    ///@brief Describes a surrounding line on the road side.
    LineSegmentSerializable roadLine;
    ///@brief Describes a surrounding line on the curb side.
    LineSegmentSerializable curbLine;
  };

  inline ::si::SlotSurroundingLines createSlotSurroundingLines()
  {
    SlotSurroundingLines m;
    (void)::eco::memset(&m, 0U, sizeof(SlotSurroundingLines));
    m.leftLine = createLineSegmentSerializable();
    m.rightLine = createLineSegmentSerializable();
    m.roadLine = createLineSegmentSerializable();
    m.curbLine = createLineSegmentSerializable();
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::SlotSurroundingLines create_default()
  {
      return ::si::createSlotSurroundingLines();
  }
}


#endif // SI_SLOT_SURROUNDING_LINES_H_