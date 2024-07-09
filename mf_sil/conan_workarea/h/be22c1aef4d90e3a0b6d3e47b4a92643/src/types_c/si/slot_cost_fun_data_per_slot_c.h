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

#ifndef SI_SLOT_COST_FUN_DATA_PER_SLOT_C_H_
#define SI_SLOT_COST_FUN_DATA_PER_SLOT_C_H_

#include "lsm_geoml/size_type_c.h"
#include "si/slot_cost_fun_data_per_opt_phase_c.h"
#include "eco/memset_c.h"

/// @brief Array of slot cost function data for every optimization phase.
typedef struct
{
    ///@range{0,3}
    ///@unit{nu}
    ///@brief Describes how many slot cost function data were already added to the array.
    LSM_GEOML_size_type actualSize;
    ///@brief Array containing the slot cost function data.
    SI_SlotCostFunDataPerOptPhase array[3];
} SI_SlotCostFunDataPerSlot;

inline SI_SlotCostFunDataPerSlot create_SI_SlotCostFunDataPerSlot(void)
{
  SI_SlotCostFunDataPerSlot m;
  (void) ECO_memset (&m, 0, sizeof(m));
  m.actualSize = 0U;
  {
    uint64 i = 0U;
    for(i = 0U; i < (sizeof(m.array) / sizeof(m.array[0])); ++i)
    {
      m.array[i] = create_SI_SlotCostFunDataPerOptPhase();
    }
  }
  return m;
}

#endif // SI_SLOT_COST_FUN_DATA_PER_SLOT_C_H_