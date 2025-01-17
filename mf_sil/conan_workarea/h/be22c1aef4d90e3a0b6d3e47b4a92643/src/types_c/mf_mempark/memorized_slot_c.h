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

#ifndef MF_MEMPARK_MEMORIZED_SLOT_C_H_
#define MF_MEMPARK_MEMORIZED_SLOT_C_H_

#include "Platform_Types.h"
#include "mf_mempark/localization_status_c.h"
#include "mf_mempark/gpssilent_offering_enabled_c.h"
#include "eco/memset_c.h"

/// The features of the list of slots
typedef struct
{
    uint8 ID;
    MF_MEMPARK_LocalizationStatus relocalizationStatus;
    MF_MEMPARK_GPSSilentOfferingEnabled gpsSilentOfferingEnabled;
    uint8 roadWidth;
} MF_MEMPARK_MemorizedSlot;

inline MF_MEMPARK_MemorizedSlot create_MF_MEMPARK_MemorizedSlot(void)
{
  MF_MEMPARK_MemorizedSlot m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_MEMPARK_MEMORIZED_SLOT_C_H_
