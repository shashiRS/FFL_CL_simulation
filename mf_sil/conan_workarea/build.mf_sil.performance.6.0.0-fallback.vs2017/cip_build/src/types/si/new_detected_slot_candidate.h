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

#ifndef SI_NEW_DETECTED_SLOT_CANDIDATE_H_
#define SI_NEW_DETECTED_SLOT_CANDIDATE_H_

#include "cml/vec2_df_pod.h"
#include "si/pb_side.h"
#include "si/slot_status.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Newly detected slot candidate.
  struct NewDetectedSlotCandidate
  {
    ///@brief One side of the opening.
    ::cml::Vec2Df_POD start;
    ///@brief Other side of the opening.
    ::cml::Vec2Df_POD end;
    ///@brief The position of the new slot candidate.
    PB_Side side;
    ///@brief The status of the new slot candidate.
    SlotStatus status;
  };

  inline ::si::NewDetectedSlotCandidate createNewDetectedSlotCandidate()
  {
    NewDetectedSlotCandidate m;
    (void)::eco::memset(&m, 0U, sizeof(NewDetectedSlotCandidate));
    m.start = ::cml::createVec2Df_POD();
    m.end = ::cml::createVec2Df_POD();
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::NewDetectedSlotCandidate create_default()
  {
      return ::si::createNewDetectedSlotCandidate();
  }
}


#endif // SI_NEW_DETECTED_SLOT_CANDIDATE_H_