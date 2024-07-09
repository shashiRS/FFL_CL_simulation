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

#ifndef SI_NEW_DETECTED_SLOT_CANDIDATE_SERIALIZABLE_H_
#define SI_NEW_DETECTED_SLOT_CANDIDATE_SERIALIZABLE_H_

#include "lsm_geoml/size_type.h"
#include "si/new_detected_slot_candidate.h"
#include "eco/memset.h"


namespace si
{

  /// @brief New detected slot candidate serializable.
  struct NewDetectedSlotCandidateSerializable
  {
    ///@range{0,25}
    ///@unit{nu}
    ///@brief Describes how many new detected slot candidates were already added to the array.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array of new detected slot candidates.
    NewDetectedSlotCandidate array[25];
  };

  inline ::si::NewDetectedSlotCandidateSerializable createNewDetectedSlotCandidateSerializable()
  {
    NewDetectedSlotCandidateSerializable m;
    (void)::eco::memset(&m, 0U, sizeof(NewDetectedSlotCandidateSerializable));
    m.actualSize = 0U;
    {
      const uint64 arraysize = (sizeof(m.array) / sizeof(m.array[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.array[i] = createNewDetectedSlotCandidate();
      }
    }
    return m;
  }

} // namespace si

namespace eco
{
  template<class T>
  inline T create_default();

  template<>
  inline ::si::NewDetectedSlotCandidateSerializable create_default()
  {
      return ::si::createNewDetectedSlotCandidateSerializable();
  }
}


#endif // SI_NEW_DETECTED_SLOT_CANDIDATE_SERIALIZABLE_H_