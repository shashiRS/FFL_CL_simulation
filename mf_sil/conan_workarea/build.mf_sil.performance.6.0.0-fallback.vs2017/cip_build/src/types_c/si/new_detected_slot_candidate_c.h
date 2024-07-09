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

#ifndef SI_NEW_DETECTED_SLOT_CANDIDATE_C_H_
#define SI_NEW_DETECTED_SLOT_CANDIDATE_C_H_

#include "cml/vec2_df_pod_c.h"
#include "si/pb_side_c.h"
#include "si/slot_status_c.h"
#include "eco/memset_c.h"

/// @brief Newly detected slot candidate.
typedef struct
{
    ///@brief One side of the opening.
    CML_Vec2Df_POD start;
    ///@brief Other side of the opening.
    CML_Vec2Df_POD end;
    ///@brief The position of the new slot candidate.
    SI_PB_Side side;
    ///@brief The status of the new slot candidate.
    SI_SlotStatus status;
} SI_NewDetectedSlotCandidate;

inline SI_NewDetectedSlotCandidate create_SI_NewDetectedSlotCandidate(void)
{
  SI_NewDetectedSlotCandidate m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.start = create_CML_Vec2Df_POD();
  m.end = create_CML_Vec2Df_POD();
  return m;
}

#endif // SI_NEW_DETECTED_SLOT_CANDIDATE_C_H_