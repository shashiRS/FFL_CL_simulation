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

#ifndef MF_HMIH_MEMORY_PARKING_INFO_C_H_
#define MF_HMIH_MEMORY_PARKING_INFO_C_H_

#include "mf_hmih/mp_slot_list_c.h"
#include "eco/memset_c.h"

/// List of memorized slots
typedef struct
{
    MF_HMIH_MpSlotList memorySlots[10];
} MF_HMIH_MemoryParkingInfo;

inline MF_HMIH_MemoryParkingInfo create_MF_HMIH_MemoryParkingInfo(void)
{
  MF_HMIH_MemoryParkingInfo m;
  (void) ECO_memset (&m, 0, sizeof(m));
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.memorySlots) / sizeof(m.memorySlots[0])); ++i)
    {
      m.memorySlots[i] = create_MF_HMIH_MpSlotList();
    }
  }
  return m;
}

#endif // MF_HMIH_MEMORY_PARKING_INFO_C_H_
