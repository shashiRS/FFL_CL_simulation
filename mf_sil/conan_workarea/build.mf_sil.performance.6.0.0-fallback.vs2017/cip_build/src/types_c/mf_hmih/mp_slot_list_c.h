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

#ifndef MF_HMIH_MP_SLOT_LIST_C_H_
#define MF_HMIH_MP_SLOT_LIST_C_H_

#include "Platform_Types.h"
#include "eco/memset_c.h"

/// 0
typedef struct
{
    ///unique identifier for slot
    uint8 slotId;
    ///to limit the maneuvering space towards the road
    float32 roadWidth;
    ///signal to indicate relocalization status
    boolean relocalizationStatus;
    ///signal to indicate automatic relocalization is enabled
    boolean gpsSilentOfferingEnabled;
} MF_HMIH_MpSlotList;

inline MF_HMIH_MpSlotList create_MF_HMIH_MpSlotList(void)
{
  MF_HMIH_MpSlotList m;
  (void) ECO_memset (&m, 0, sizeof(m));
  return m;
}

#endif // MF_HMIH_MP_SLOT_LIST_C_H_
