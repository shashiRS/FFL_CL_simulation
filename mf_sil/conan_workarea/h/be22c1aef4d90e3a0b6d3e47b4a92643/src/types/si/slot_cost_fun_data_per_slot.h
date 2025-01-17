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

#ifndef SI_SLOT_COST_FUN_DATA_PER_SLOT_H_
#define SI_SLOT_COST_FUN_DATA_PER_SLOT_H_

#include "lsm_geoml/size_type.h"
#include "si/slot_cost_fun_data_per_opt_phase.h"
#include "eco/memset.h"


namespace si
{

  /// @brief Array of slot cost function data for every optimization phase.
  struct SlotCostFunDataPerSlot
  {
    ///@range{0,3}
    ///@unit{nu}
    ///@brief Describes how many slot cost function data were already added to the array.
    ::lsm_geoml::size_type actualSize;
    ///@brief Array containing the slot cost function data.
    SlotCostFunDataPerOptPhase array[3];
  };

  inline ::si::SlotCostFunDataPerSlot createSlotCostFunDataPerSlot()
  {
    SlotCostFunDataPerSlot m;
    (void)::eco::memset(&m, 0U, sizeof(SlotCostFunDataPerSlot));
    m.actualSize = 0U;
    {
      const uint64 arraysize = (sizeof(m.array) / sizeof(m.array[0]));
      for(uint64 i = 0U; i < arraysize; ++i)
      {
        m.array[i] = createSlotCostFunDataPerOptPhase();
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
  inline ::si::SlotCostFunDataPerSlot create_default()
  {
      return ::si::createSlotCostFunDataPerSlot();
  }
}


#endif // SI_SLOT_COST_FUN_DATA_PER_SLOT_H_
